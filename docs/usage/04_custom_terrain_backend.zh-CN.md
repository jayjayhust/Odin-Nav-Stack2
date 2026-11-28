# 04 · 增加自定义地形识别器

[English](04_custom_terrain_backend.md) | **中文**

`terrain_recognizer` 自带两个可用的几何后端 —— `stair_gait`（高度剖面踏步检测器）和 `ramp_slope`（连续斜坡的平面拟合）—— 以及一个只有接口、没有实现的 `dino_v3` 空壳。本文是一条完整路径：从"我有一个模型"到"整栈读到了我的结论"。内容涵盖槽位职责划分、后端契约、可直接编译的骨架、构建与配置接线、如何与已有识别器并存，以及没有机器人时怎么测。

如果你还没配置过这个槽位，先读 [02 · 参数调优 § 8](02_parameter_tuning.zh-CN.md)。下文路径若未特别说明，均相对于 `ros_ws/`。

## 1. 谁负责什么

节点和后端的分工是刻意设计过的。把逻辑放错边，就是"我的模型能跑，但导航栈不理会"的典型原因。

| 职责 | 归属 |
|---|---|
| 订阅点云、转换到 `base_frame`、按 `detect_rate_hz` 定时 tick | 节点 |
| 在后端运行之前，把点云裁剪到每个实例的 `roi` | 节点 |
| 去抖（`confirm_frames`）、latched 发布、聚合成 `/terrain_recognizer/scene` | 节点 |
| 给 NeuPAN 脱困门控用的单值 `/stair_gait/status` JSON | 节点 |
| 把裁剪后的点云（以及它自己额外订阅的输入）变成**一个 `TerrainResult`** | 后端 |

所以后端从不发布消息，不碰槽位拥有的 ROS 话题，也不决定自己什么时候被调用。它是一个带状态的函数。

## 2. 契约

见 [`include/terrain_recognizer/terrain_backend.h`](../../ros_ws/src/terrain_recognizer/include/terrain_recognizer/terrain_backend.h)：

| 虚函数 | 调用时机 | 你要做的事 |
|---|---|---|
| `std::string name() const` | 启动、日志、每条消息 | **返回与注册时完全相同的字符串**，它会被写进 `TerrainObservation.backend` |
| `void configure(ros::NodeHandle& pnh)` | 每个实例构造时一次，在首次 `observe()` 之前 | 读 `~` 参数、加载模型、订阅额外输入（相机、地图）。可以阻塞几秒，只在启动阶段发生 |
| `std::string describe() const` | 一次，启动日志 | 一行，写关键的调参项。它按实例打印，别太长 |
| `void observe(const pcl::PointCloud<pcl::PointXYZ>& cloud_base, const std_msgs::Header& header)` | 每个 tick，在节点的定时器线程上 | 更新缓存的结论。`cloud_base` 已在 `base_frame` 下，且已按实例 `roi` 裁剪过 |
| `TerrainResult result() const` | 紧跟 `observe()`，同一线程 | 返回缓存结论。要便宜、const、无 I/O |

`TerrainResult` 就是全部输出面：

| 字段 | 含义 | 规则 |
|---|---|---|
| `type` | `flat`、`stair_up`、`stair_down`、`ramp_up`、`ramp_down`、`landing`、`unknown` | 消费者依据的类别。`unknown` 是一个合法结论 |
| `legacy_state` | `flat`、`up`、`down`、`unknown` | **必填且必须与 `type` 一致**。这是 NeuPAN 脱困门控唯一读的字段：`stair_up → up`、`stair_down → down`、`ramp_*`/`landing → flat`（分不清就 `unknown`） |
| `confidence` | `0..1`，后端自定义 | `/stair_gait/status` 归约器用它打破平票，所以要求你自己的各帧之间可比，不必跨后端可比 |
| `steps`、`rise_m`、`span_m` | 楼梯几何 | 模型能回归就填，否则留零 |
| `valid` | "这是我当前的结论" | `false` = **保持上一条**（节点不发布，并把该实例从 scene 里去掉）。暖机阶段或输入缺失时用它 —— 绝不能在猜的时候报 `valid=true` |
| `region` | `base_frame` 下的 `PolygonStamped` 区域 | 节点会再裁到实例 `roi`。模型无法定位时可留空 |

五条值得读两遍的硬规矩：

1. **不确定时 `valid=false`。** 一个刚丢了相机的学习型模型不能继续发上一个类别。冻结成 `valid=false`，让消费者保持自己的状态。
2. **`legacy_state` 不是可选项。** 只填 `type` 的后端，会让脱困门控永远看不到楼梯。
3. **绝不阻塞。** 节点用的是单线程 `ros::spin()`，慢的 `observe()` 会卡住点云订阅，也卡住其他所有实例。模型比一个周期慢的话，就缓存上一条结论先返回，新结论就绪前保持 `valid=false` —— `dino_v3` 空壳的头注释写明了这种形状。
4. **每个实例一个后端对象。** `createTerrainBackend()` 对每个声明的实例各调一次，所以两个区域用同一个后端名，会得到两个独立对象。在 `configure()` 里加载 200 MB 编码器，两个区域就是 400 MB：用一个函数内 `static` 共享（注册表自己对它的 map 就是这么做的）。
5. **不要发布。** 话题、latch、scene 消息都归节点。

## 3. 第 1 步 · 复制空壳

```bash
cp ros_ws/src/terrain_recognizer/src/backends/dino_v3_backend.cpp \
   ros_ws/src/terrain_recognizer/src/backends/my_curb_backend.cpp
```

下面的骨架是一个完整、可编译的后端，回答"近处地面是不是一个水平面？" —— 在你写真正的模型之前，它足够让你自己的结论走通整栈。把 `classify()` 换成你的推理调用即可。

```cpp
// Copyright 2026 Manifold Tech Ltd.
//
// My curb backend: <one sentence on the model and its input>.
#include <algorithm>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

#include "terrain_recognizer/terrain_backend.h"

namespace terrain_recognizer {
namespace {

class MyCurbBackend : public TerrainBackend {
 public:
  std::string name() const override { return "my_curb"; }

  void configure(ros::NodeHandle& pnh) override {
    // Prefix your keys with the backend name. Every instance shares the node's
    // private namespace, and other backends may already use a bare `model_path`.
    pnh.param<std::string>("my_curb/model_path", model_path_, std::string(""));
    pnh.param<double>("my_curb/plane_tol_m", plane_tol_m_, 0.05);
    ROS_INFO("[terrain_recognizer] my_curb model_path='%s'", model_path_.c_str());
  }

  std::string describe() const override {
    return "my_curb plane_tol=" + std::to_string(plane_tol_m_) + "m";
  }

  void observe(const pcl::PointCloud<pcl::PointXYZ>& cloud_base,
               const std_msgs::Header& header) override {
    (void)header;  // use it if you need to sync a camera frame
    result_ = classify(cloud_base);
  }

  TerrainResult result() const override { return result_; }

 private:
  TerrainResult classify(const pcl::PointCloud<pcl::PointXYZ>& cloud) const {
    TerrainResult out;  // defaults: type/legacy = "unknown", valid = false
    if (cloud.size() < 50) return out;  // no data yet: keep the previous verdict

    double z_min = 1.0e9, z_max = -1.0e9;
    int in_range = 0;
    for (const auto& p : cloud.points) {
      if (p.x < 0.3 || p.x > 1.5) continue;  // near ground only
      z_min = std::min(z_min, static_cast<double>(p.z));
      z_max = std::max(z_max, static_cast<double>(p.z));
      ++in_range;
    }
    if (in_range < 50) return out;  // crop looked empty: keep the previous verdict
    if (z_max - z_min <= plane_tol_m_) {
      out.type = "flat";
      out.legacy_state = "flat";
      out.confidence = 1.0 - (z_max - z_min) / plane_tol_m_;
      out.valid = true;
    } else {
      // Looked and cannot classify: a real verdict, not "no data".
      out.valid = true;
    }
    return out;
  }

  std::string model_path_;
  double plane_tol_m_ = 0.05;
  TerrainResult result_;
};

TerrainBackendPtr makeMyCurbBackend() { return TerrainBackendPtr(new MyCurbBackend()); }

// Static registrar: the registry is a function-local static, so this is safe
// from any translation unit at any point during static initialisation.
struct Registrar {
  Registrar() { registerTerrainBackend("my_curb", &makeMyCurbBackend); }
} g_registrar;

}  // namespace
}  // namespace terrain_recognizer
```

## 4. 第 2 步 · 构建接线

`ros_ws/src/terrain_recognizer/CMakeLists.txt` —— 两行：

```cmake
add_executable(terrain_recognizer_node
  src/terrain_recognizer_node.cpp
  src/terrain_backend_factory.cpp
  src/backends/stair_gait_backend.cpp
  src/backends/dino_v3_backend.cpp
  src/backends/my_curb_backend.cpp          # <-- add
)

target_link_libraries(terrain_recognizer_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  stair_detector
  onnxruntime                                # <-- only what your model needs
)
```

两条值得照抄的约定：

- **重依赖或可选依赖保持可选。** `stair_gait/CMakeLists.txt` 用 `find_package(unitree_sdk2 QUIET)`，只有在装了 SDK 时才构建依赖它的节点；缺一个运行时，代价就是少一个后端，而不是整个工作空间编译失败。用 `if(onnxruntime_FOUND)` 做同样的事。
- **可复用的模型上提成独立包。** 如果推理在槽位之外也有用（比如仿真也要用），把算法放在自己的 catkin 包里导出库，再在这里链接 —— 正如 `stair_gait` 为本节点导出 `stair_detector`。后端应当保持为薄适配层。

新增的 ROS 依赖（图像传输、`cv_bridge`、消息包）还必须写进 `ros_ws/src/terrain_recognizer/package.xml`，否则换台机器干净构建就会失败。

## 5. 第 3 步 · 参数

默认值放在 `ros_ws/src/terrain_recognizer/config/my_curb.yaml`。键从节点的私有命名空间读取，所以 `my_curb/` 前缀能把它们与其他后端隔开：

```yaml
# my_curb backend defaults. Loaded only when this backend is selected.
my_curb/model_path: ""
my_curb/plane_tol_m: 0.05
```

何时加载这个文件由 launch 决定（`terrain_recognizer.launch`，五层配置里的第 2 层）：

```xml
<rosparam if="$(eval arg('backend') == 'my_curb')"
          command="load" file="$(find terrain_recognizer)/config/my_curb.yaml"/>
```

有一条必须讲清楚的限制：**这行只在单实例简写模式下生效。** 用 `terrain_recognizer.instances` 时，节点级的 `backend` 参数是空的（或是 profile 里的默认后端），因此不会加载按后端划分的包内文件 —— 把这些键写进机器人自己的文件（第 3 层，永远生效）：

```yaml
# robots/<robot>/robot.yaml
terrain_recognizer:
  backend: stair_gait
  config: terrain_recognizer/my_params.yaml      # loaded over both layers above
```

## 6. 第 4 步 · 选择它

`robots/<robot>/robot.yaml`，两种形态之一：

```yaml
# (a) single recognizer, historical topics
terrain_recognizer:
  backend: my_curb
```

```yaml
# (b) one instance among several; /terrain_recognizer/right/status
terrain_recognizer:
  instances: terrain_recognizer/instances.yaml

# instances.yaml
instances:
  front:
    backend: stair_gait
    roi: {x_min: 0.6, x_max: 3.2, y_min: -0.6, y_max: 0.6}
  right:
    backend: my_curb
    legacy: false                                  # never votes on /stair_gait/status
    roi: {x_min: 0.4, x_max: 1.6, y_min: -1.5, y_max: -0.3}
```

启动前先确认 bringup 到底会收到什么：

```bash
python3 scripts/lib/robot_profile.py --terrain-backend robots/my_robot/robot.yaml
python3 scripts/lib/robot_profile.py --terrain-instances robots/my_robot/robot.yaml
```

## 7. 第 5 步 · 验证

```bash
catkin_make --pkg terrain_recognizer
roslaunch ons2_bringup go2.launch map_file_name:=NAME \
  neupan_config:=$PWD/robots/my_robot/neupan/config.yaml \
  $(python3 scripts/lib/robot_profile.py --ros1-bringup-args robots/my_robot/robot.yaml)

rostopic echo -n1 /terrain_recognizer/status       # single-shorthand mode
rostopic echo -n1 /terrain_recognizer/right/status # instance mode
rostopic echo -n1 /terrain_recognizer/scene        # all instances, one message
rostopic echo -n1 /stair_gait/status               # the reduced legacy JSON
rosnode info /terrain_recognizer                   # confirm the publishers exist
```

启动日志会按实例列出各自的 `describe()`，例如：

```
[terrain_recognizer] instances(2) front:stair_gait roi x=[0.60,3.20] ... | right:my_curb plane_tol=0.05m cloud=/odin1/cloud_slam base=base_link rate=10.0Hz
```

如果启动时节点报 `unknown terrain backend 'my_curb' (available: dino_v3, ramp_slope, stair_gait)` 退出，说明你的源文件没进可执行文件，或注册对象没被链接进来 —— 见 § 9 第一行。

## 8. 没有机器人时怎么测

这个契约可以离线测，因为 `observe()` 只要求一份 `base_frame` 下的点云。

**单元测试** —— 直接喂合成几何：

```cpp
// ros_ws/src/terrain_recognizer/test/test_my_curb_backend.cpp
#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "terrain_recognizer/terrain_backend.h"

TEST(MyCurbBackend, FlatPlaneIsFlat) {
  terrain_recognizer::TerrainBackendPtr backend =
      terrain_recognizer::createTerrainBackend("my_curb");
  ASSERT_TRUE(backend != nullptr);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (double x = 0.3; x < 1.5; x += 0.02) {
    for (double y = -0.6; y < 0.6; y += 0.02) cloud.emplace_back(x, y, 0.0);
  }
  backend->observe(cloud, std_msgs::Header());
  EXPECT_TRUE(backend->result().valid);
  EXPECT_EQ(backend->result().type, "flat");
  EXPECT_EQ(backend->result().legacy_state, "flat");
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_my_curb_backend", ros::init_options::AnonymousName);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
```

```cmake
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_my_curb_backend
    test/test_my_curb_backend.cpp
    src/terrain_backend_factory.cpp
    src/backends/my_curb_backend.cpp)          # link the TU, don't rely on the exe
  target_link_libraries(test_my_curb_backend ${catkin_LIBRARIES} ${PCL_LIBRARIES})
endif()
```

**无硬件端到端** —— 直接以节点自己的坐标系发布合成点云，这样连 TF 都不需要（`base_frame:=odin1_base_link` 配 `frame_id: odin1_base_link`，查询即恒等变换）：

```python
#!/usr/bin/env python3
"""Flat ground with a 0.18 m step at x=1.0, published as if it came from Odin1."""
import numpy as np
import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2

rospy.init_node('fake_cloud')
pub = rospy.Publisher('/odin1/cloud_slam', PointCloud2, queue_size=1)
xs, ys = np.meshgrid(np.arange(0.3, 2.0, 0.02), np.arange(-0.6, 0.6, 0.02))
zs = np.where(xs > 1.0, 0.18, 0.0)
pts = np.stack([xs.ravel(), ys.ravel(), zs.ravel()], axis=1).tolist()
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    header = rospy.Header(stamp=rospy.Time.now(), frame_id='odin1_base_link')
    pub.publish(point_cloud2.create_cloud_xyz32(header, pts))
    rate.sleep()
```

```bash
rosrun terrain_recognizer terrain_recognizer_node _base_frame:=odin1_base_link _backend:=my_curb
rostopic echo /terrain_recognizer/status
```

**回放** —— 模型做出来以后，录下来的现场数据比合成几何有用得多：在机器人上 `rosbag record /odin1/cloud_slam /tf /tf_static`，本地 `rosbag play --clock` 加同一个节点即可，栈里其他节点都不用起。

## 9. 排障

| 现象 | 最可能的原因 |
|---|---|
| 节点退出：`unknown terrain backend 'x'` | 源文件没加进 `add_executable`，或注册对象在静态库里被链接器丢弃。看消息里的 `available:` 列表 |
| `name()` 与注册名不一致 | 消息里写 `backend: "x"`，消费者在找 `"y"`。原样返回注册字符串 |
| 一直看不到结论 | `valid` 一直是 `false`；`confirm_frames` 去抖还没到数；ROI 裁剪后是空点云（实例 `roi` 与点云不重叠）；`cloud_topic` / `base_frame` 配错（看 TF 警告） |
| 脱困门控对楼梯不再反应 | `legacy_state` 没填，或与 `type` 不一致；或该实例 `legacy: false`；或它的区域并没有真正覆盖前方走廊（`legacy_corridor_half_width`） |
| 所有实例数字一模一样 | 参数是节点级的：不同后端的同名键互相覆盖。加后端名前缀 |
| 单实例话题有数据，`/terrain_recognizer/scene` 却是空的 | scene 只在当帧至少一个实例有结论时发布；后加入的订阅者拿到的是 latched 的最后一条非空消息 |
| 新模型一上，导航频率掉一半 | `observe()` 阻塞了节点唯一的 spinner 线程。先缓存返回，把推理放到自己的工作线程 |
| `rostopic hz` 看到结论反复跳 | 去抖键是 `type + "/" + legacy_state`，`legacy_state` 不稳定会每帧重置计数 |

## 10. 参考实现

- [`src/backends/stair_gait_backend.cpp`](../../ros_ws/src/terrain_recognizer/src/backends/stair_gait_backend.cpp) —— 几何参考实现：包装一个无 ROS 依赖的库，把 `up/down/flat` 映射到槽位词汇，并把它的 ROI 作为 `region` 多边形报出。
- [`src/backends/ramp_slope_backend.cpp`](../../ros_ws/src/terrain_recognizer/src/backends/ramp_slope_backend.cpp) —— 自包含的几何参考实现（不依赖库，~280 行）：自己建剖面、做直线拟合，并示范三个值得照抄的约定 —— 后端前缀参数（`ramp_slope/*`）让多个后端共用同一个节点命名空间；`legacy_state` 按**安全**而不是字面真实来选（坡道报 `flat`，所以它永远按不住脱困门控）；证据门槛把 `valid=false`（"还没有结论"）与 `unknown`（"看了，但判不了"）分开。
- [`src/backends/dino_v3_backend.cpp`](../../ros_ws/src/terrain_recognizer/src/backends/dino_v3_backend.cpp) —— 学习型原型：参数面已经定好（`model_path`、`image_topic`、`device`、`input_size`），头注释写明了真实编码器要做的每一件事，包括延迟与 `legacy_state` 义务。
- [ONS2 架构 § 5.6](../architecture/ONS2_ARCHITECTURE.md#56-terrain-recognizer-terrain_recognizer) —— 槽位在整栈中的位置，以及节点为什么从不直接驱动步态。
- [路线图](../ROADMAP.zh-CN.md) —— 逐实例参数、学习型后端的护栏、第一个 DINOv3 原型的阶段划分。

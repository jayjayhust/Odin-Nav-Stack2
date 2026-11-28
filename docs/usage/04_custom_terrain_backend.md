# 04 · Add a Custom Terrain Recognizer

**English** | [中文](04_custom_terrain_backend.zh-CN.md)

`terrain_recognizer` ships two working geometric backends — `stair_gait` (height-profile tread detector) and `ramp_slope` (plane fit for continuous slopes) — plus one interface-complete stub, `dino_v3`. This guide is the whole path from "I have a model" to "the stack reads my verdicts": what the slot owns, the backend contract, a compiling skeleton, the build/config wiring, how to run it next to an existing recognizer, and how to test it without a robot.

Read [02 · Parameter Tuning § 8](02_parameter_tuning.md#8-terrain-recognizer-slot-terrain_recognizer) first if you have not configured the slot at all. Paths below are relative to `ros_ws/` unless stated otherwise.

## 1. Who owns what

The node and the backend have deliberately different jobs. Putting logic on the wrong side is the usual source of "my model works but the stack ignores it".

| Concern | Owner |
|---|---|
| Cloud subscription, TF into `base_frame`, one timer tick per `detect_rate_hz` | node |
| Cropping the cloud to each instance's `roi` before the backend runs | node |
| Debounce (`confirm_frames`), latched publishing, `/terrain_recognizer/scene` aggregation | node |
| The single reduced `/stair_gait/status` JSON consumed by NeuPAN's escape gate | node |
| Turning the cropped cloud (plus anything it subscribes itself) into **one `TerrainResult`** | backend |

So a backend never publishes, never touches ROS topics owned by the slot, and never decides when it is asked. It is a function with state.

## 2. The contract

From [`include/terrain_recognizer/terrain_backend.h`](../../ros_ws/src/terrain_recognizer/include/terrain_recognizer/terrain_backend.h):

| Virtual | Called | What you implement |
|---|---|---|
| `std::string name() const` | startup, log, every message | Return the **same string you registered**. It becomes `TerrainObservation.backend`. |
| `void configure(ros::NodeHandle& pnh)` | once per instance, before the first `observe()` | Read `~` parameters, load the model, subscribe to extra inputs (camera, maps). May block for a few seconds; runs at startup only. |
| `std::string describe() const` | once, startup log | One line with the knobs that matter. Printed per instance, so keep it short. |
| `void observe(const pcl::PointCloud<pcl::PointXYZ>& cloud_base, const std_msgs::Header& header)` | every tick, on the node's timer thread | Replace your cached verdict. `cloud_base` is already in `base_frame` and already cropped to the instance's `roi`. |
| `TerrainResult result() const` | immediately after `observe()`, same thread | Return the cached verdict. Cheap, const, no I/O. |

`TerrainResult` is the whole output surface:

| Field | Meaning | Rules |
|---|---|---|
| `type` | `flat`, `stair_up`, `stair_down`, `ramp_up`, `ramp_down`, `landing`, `unknown` | The class consumers key on. `unknown` is a legitimate verdict. |
| `legacy_state` | `flat`, `up`, `down`, `unknown` | **Required and must agree with `type`.** This is the only field NeuPAN's escape gate reads: `stair_up → up`, `stair_down → down`, `ramp_*`/`landing → flat` (or `unknown` if you cannot tell). |
| `confidence` | `0..1`, backend-defined | Used to break ties inside the `/stair_gait/status` reducer, so make it comparable across your own frames, not necessarily across backends. |
| `steps`, `rise_m`, `span_m` | Stair geometry | Fill from the model if it regresses them; leave zero otherwise. |
| `valid` | "this is my current verdict" | `false` = *keep the previous one* (the node publishes nothing and drops the instance from the scene). Use it while warming up or when the input is missing — never report `valid=true` with a guessed class. |
| `region` | `PolygonStamped` region in `base_frame` | The node clips it to the instance `roi`. Empty is allowed if your model cannot localize what it saw. |

Five hard rules that are worth reading twice:

1. **`valid=false` while unsure.** A learned model that just lost its camera must not keep publishing the last class. Freeze `valid=false` and consumers hold their state.
2. **`legacy_state` is not optional.** A backend that only fills `type` silently stops the escape gate from ever seeing a staircase.
3. **Never block.** The node runs a single-threaded `ros::spin()`, so a slow `observe()` stalls the cloud subscription and every other instance too. If your model is slower than one period, cache the last verdict, return it, and set `valid=false` until the new one is ready — see the `dino_v3` stub header for the intended shape.
4. **One backend object per instance.** `createTerrainBackend()` is called once per declared instance, so two regions declared with the same backend key give you two independent objects. A 200 MB encoder loaded in `configure()` is therefore 400 MB with two regions: share it through a function-local `static` (the registry does exactly that for its own map).
5. **Don't publish.** Topics, latches and the scene message belong to the node.

## 3. Step 1 · Copy the stub

```bash
cp ros_ws/src/terrain_recognizer/src/backends/dino_v3_backend.cpp \
   ros_ws/src/terrain_recognizer/src/backends/my_curb_backend.cpp
```

The skeleton below is a complete, compiling backend that answers "is the near ground one horizontal plane?" — enough to see your own verdicts flowing through the whole stack before you write the real model. Replace `classify()` with your inference call.

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

## 4. Step 2 · Build wiring

`ros_ws/src/terrain_recognizer/CMakeLists.txt` — two lines:

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

Two conventions worth copying:

- **Heavy or optional dependencies stay optional.** `stair_gait/CMakeLists.txt` does `find_package(unitree_sdk2 QUIET)` and builds the SDK-dependent node only when it is present; a missing runtime then costs you one backend, not the whole workspace. Do the same with `if(onnxruntime_FOUND)`.
- **Upstream a reusable model as a separate package.** If the inference is useful outside this slot (e.g. it also feeds the simulator), keep the algorithm in its own catkin package exporting a library and link that here, exactly as `stair_gait` exports `stair_detector` for this node. Backends should stay thin adapters.

New ROS dependencies (image transport, `cv_bridge`, a message package) must also be added to `ros_ws/src/terrain_recognizer/package.xml`, otherwise a clean build on another machine fails.

## 5. Step 3 · Parameters

Ship defaults in `ros_ws/src/terrain_recognizer/config/my_curb.yaml`. Keys are read from the node's private namespace, so a `my_curb/` prefix keeps them out of the other backends' way:

```yaml
# my_curb backend defaults. Loaded only when this backend is selected.
my_curb/model_path: ""
my_curb/plane_tol_m: 0.05
```

The launch file decides when that file is loaded (`terrain_recognizer.launch`, layer 2 of 5):

```xml
<rosparam if="$(eval arg('backend') == 'my_curb')"
          command="load" file="$(find terrain_recognizer)/config/my_curb.yaml"/>
```

One caveat, stated plainly because it surprises people: **that line only fires in single-shorthand mode.** With `terrain_recognizer.instances` the node-level `backend` argument is empty (or the profile's default backend), so per-backend package files are not loaded — put the same keys in the robot's own file instead, which is layer 3 and always applies:

```yaml
# robots/<robot>/robot.yaml
terrain_recognizer:
  backend: stair_gait
  config: terrain_recognizer/my_params.yaml      # loaded over both layers above
```

## 6. Step 4 · Select it

`robots/<robot>/robot.yaml`, one of two shapes:

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

Check what the bringup will actually receive before launching:

```bash
python3 scripts/lib/robot_profile.py --terrain-backend robots/my_robot/robot.yaml
python3 scripts/lib/robot_profile.py --terrain-instances robots/my_robot/robot.yaml
```

## 7. Step 5 · Verify

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

The startup log lists every instance with its `describe()` string, e.g.

```
[terrain_recognizer] instances(2) front:stair_gait roi x=[0.60,3.20] ... | right:my_curb plane_tol=0.05m cloud=/odin1/cloud_slam base=base_link rate=10.0Hz
```

If the node dies during startup with `unknown terrain backend 'my_curb' (available: dino_v3, ramp_slope, stair_gait)`, your file is not in the executable or the registrar is not linked — see the first row of § 9.

## 8. Testing without the robot

The contract is testable offline because `observe()` only needs a cloud in `base_frame`.

**Unit test** — feed synthetic geometry:

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

**End-to-end without hardware** — publish a synthetic cloud in the node's own frame, so no TF is needed (`base_frame:=odin1_base_link` + `frame_id: odin1_base_link` is the identity lookup):

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

**Replay** — a recorded site is better than synthetic geometry once the model is real: `rosbag record /odin1/cloud_slam /tf /tf_static` on the robot, then play it locally with `--clock` and the same node. Nothing else in the stack has to run.

## 9. Troubleshooting

| Symptom | Likely cause |
|---|---|
| Node exits: `unknown terrain backend 'x'` | Source missing from `add_executable`, or the registrar sits in a static library the linker dropped. Check the `available:` list in the message. |
| `name()` differs from the registered key | Messages claim `backend: "x"`, filters look for `"y"`. Return the registration string verbatim. |
| Verdicts never appear | `valid` stays `false`; `confirm_frames` debounce not yet reached; ROI crop is empty (instance `roi` and the cloud do not overlap); wrong `cloud_topic` / `base_frame` (watch the TF warning). |
| Escape gate stops reacting to stairs | `legacy_state` not set, or set inconsistently with `type`; or the instance has `legacy: false`; or its region does not really overlap the forward corridor (`legacy_corridor_half_width`). |
| All instances report identical numbers | Node-global parameters: unprefixed keys collide between backends. Prefix with the backend name. |
| `/terrain_recognizer/scene` is empty while a per-instance topic works | Scene is only published when at least one instance has a verdict this frame; the late joiner gets the latched last non-empty scene. |
| New model halves the navigation rate | `observe()` blocks the node's only spinner thread. Cache and return; move inference to your own worker. |
| `rostopic hz` shows verdicts flapping | Debounce key is `type + "/" + legacy_state`; an unstable `legacy_state` restarts the counter every frame. |

## 10. Reference implementations

- [`src/backends/stair_gait_backend.cpp`](../../ros_ws/src/terrain_recognizer/src/backends/stair_gait_backend.cpp) — the geometric reference: wraps a ROS-free library, maps `up/down/flat` to the slot vocabulary, reports its ROI as the `region` polygon.
- [`src/backends/ramp_slope_backend.cpp`](../../ros_ws/src/terrain_recognizer/src/backends/ramp_slope_backend.cpp) — the self-contained geometric reference (no library, ~280 lines): builds the profile itself, fits a line, and shows three conventions worth copying — backend-prefixed parameters (`ramp_slope/*`) so several backends share one node namespace, `legacy_state` chosen by what is *safe* rather than what is literally true (a ramp reports `flat` so it can never hold the escape gate), and evidence gates that return `valid=false` ("no verdict yet") separately from `unknown` ("looked, cannot classify").
- [`src/backends/dino_v3_backend.cpp`](../../ros_ws/src/terrain_recognizer/src/backends/dino_v3_backend.cpp) — the learned prototype: parameter surface already fixed (`model_path`, `image_topic`, `device`, `input_size`), and the header comment states exactly what a real encoder must do, including the latency and `legacy_state` obligations.
- [ONS2 Architecture § 5.6](../architecture/ONS2_ARCHITECTURE.md#56-terrain-recognizer-terrain_recognizer) — where the slot sits in the stack and why the node never actuates.
- [ROADMAP](../ROADMAP.md) — phases for per-instance parameters, learned backend guardrails and the first DINOv3 prototype.

repo: /home/user/Doubao/chats/38449032734391554/Odin-Nav-Stack2
branch: feature/go2-multifloor
commit: 80b3d61 (local only, NOT pushed)

新增/修改清单
==============
新增包 ros_ws/src/floor_manager/
  package.xml / CMakeLists.txt
  include/floor_manager/map_io.h
  src/map_io.cpp                 yaml+pgm -> OccupancyGrid (P2/P5, 行翻转, negate/threshold)
  src/floor_manager_node.cpp     切层原子操作状态机 (624 行)
  launch/floor_manager.launch
  test/map_io_offline_check.py   离线回归（已跑通）

接口契约 ros_ws/src/nav_interfaces/
  msg/Waypoint.msg        + string floor（末尾追加，空=当前层）
  msg/FloorStatus.msg     新增 current_floor/target_floor/switching/detail
  srv/SwitchFloor.srv     新增
  srv/ArrivedAtFloor.srv  新增
  CMakeLists.txt          已注册

mission_manager/
  include/.../mission_manager.h   State::FLOOR_TRANSIT + 5 方法 + multifloor 成员
  src/mission_manager_node.cpp    见下
  launch/mission_manager.launch   enable_multifloor/floors_config/initial_floor/...

ons2_bringup/launch/go2.launch    + enable_map_server 开关（跨层模式必须 false）

sliding_map/
  include/sliding_map.h + src/sliding_map.cpp   /sliding_map_node/clear
  CMakeLists.txt + package.xml                  + std_srvs 依赖

scripts/
  ons2_odin_restart.sh      process|systemd, --dry-run, 30s 内验证进程
  ons2_go2_multifloor.sh    封装 MULTIFLOOR=1 ENABLE_MAP_SERVER=0
  ons2_go2.sh               + MULTIFLOOR/INITIAL_FLOOR 分支 + floor_manager 窗口

docs/multifloor/
  README.zh-CN.md   建图->编译->启动->下达任务->验证->排障
  design.zh-CN.md   接口契约 + 状态机 + 11 步切层时序 + 设计取舍

robots/go2/floors.yaml   4 层示例，transitions: stairs(manual) 优先, lift(manual) 兜底

mission_manager 关键改动
========================
1) 构造：读 enable_multifloor/floor_status_topic/initial_floor/
   transition_arrive_tolerance_m/transition_wait_timeout_sec，解析 ~floors 建 floor_elevators_，
   订阅 /floor_manager/status
2) currentGoalPose(): floor_transit_active_ ? transit_goal_ : waypointPose(...)
   已被 planCurrentWaypoint / processPeriodicReplan / recovery shadow /
   distanceToTarget / publishStatus 复用（避免跨层用旧层坐标）
3) startCurrentWaypoint() 开头：target_floor != current_floor_ ->
   已在电梯口(<=0.30m)则 enterElevatorTransit，否则规划到电梯口
4) ARRIVED 分支前：floor_transit_active_ -> enterElevatorTransit 并 return（不做对齐/停留）
5) process() 最前：FLOOR_TRANSIT -> processElevatorTransit()
6) processElevatorTransit(): 等 !switching && current_floor==target ->
   更新 current_floor_、home_pose_valid_=false、续跑原航点
7) cancel/reset/startMission 清理 floor_transit_active_ 与 target_floor_

静态校验（均可复现）
====================
XML 4 个 launch 文件 well-formed ........ PASS
floors.yaml 结构断言 ..................... PASS
roslaunch arg 引用无缺失 ................. PASS
C++ 括号平衡（去注释/字符串后） .......... PASS
PGM 行翻转 + 阈值 + negate + origin ...... PASS
bash -n 三个脚本 ......................... PASS
c++ 编译 ................................. 未做（本机无 ROS/Noetic）

待用户在 Jetson 上执行
======================
git push origin feature/go2-multifloor   # 本机 GITHUB_TOKEN 为空，未能推送
cd ros_ws && catkin_make -DCMAKE_BUILD_TYPE=Release \
  -DCATKIN_WHITELIST_PACKAGES="nav_interfaces;odin_ros_driver;map_planner;sliding_map;neupan_ros;mission_manager;atomic_move;floor_manager;octo_planner_ros1;stair_gait"

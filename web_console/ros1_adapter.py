"""ROS 1 adapter: wires the map, pose, mission, safety and camera topics/services to MonitorStateHub."""

import math
import threading
import time
from array import array

from web_console.core import yaw_from_quaternion

START_MISSION_SERVICE = "/waypoint_manager/start_mission"
FRONT_SECTOR_RAD = 0.7      # ~40 deg total window around the first beam of the scan
MAX_SCAN_SAMPLES = 20


def run(args, hub):
    import rospy
    import tf2_ros
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry, OccupancyGrid
    from sensor_msgs.msg import CompressedImage, LaserScan
    from std_msgs.msg import Bool, Empty, String
    from std_srvs.srv import Trigger

    try:
        from nav_interfaces.msg import MissionEvent, MissionStatus, Waypoint
        from nav_interfaces.srv import StartMission
    except ImportError:
        MissionEvent = MissionStatus = Waypoint = StartMission = None

    rospy.init_node("web_console", anonymous=False)
    tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
    tf2_ros.TransformListener(tf_buffer)
    img_lock = threading.Lock()
    img_sub = {"handle": None}
    scan_lock = threading.Lock()
    scan_state = {"latest": None}

    def on_map(msg):
        info = msg.info
        hub.console.set_map(info.width, info.height, info.resolution,
                            info.origin.position.x, info.origin.position.y, array("b", msg.data))

    def on_odom(msg):
        p, q, v, w = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist.linear, msg.twist.twist.angular
        hub.on_odom(p.x, p.y, v.x, v.y, w.z, yaw_from_quaternion(q.x, q.y, q.z, q.w))

    def on_event(msg):
        hub.on_mission_event({
            "event": msg.event,
            "waypoint_idx": int(msg.waypoint_idx),
            "waypoint_name": msg.waypoint_name,
            "timestamp": float(msg.timestamp),
            "message": msg.message,
        })

    def on_scan(msg):
        with scan_lock:
            scan_state["latest"] = msg

    def on_status(msg):
        tgt = msg.current_target
        hub.on_mission_status({
            "state": msg.state,
            "mission_id": msg.mission_id,
            "current_waypoint_idx": int(msg.current_waypoint_idx),
            "total_waypoints": int(msg.total_waypoints),
            "distance_to_target_m": float(msg.distance_to_target_m),
            "progress_percent": float(msg.progress_percent),
            "dwell_remaining_sec": float(msg.dwell_remaining_sec),
            "target": {"x": tgt.position.x, "y": tgt.position.y,
                       "yaw": yaw_from_quaternion(tgt.orientation.x, tgt.orientation.y,
                                                  tgt.orientation.z, tgt.orientation.w)},
        })

    def on_pose_tick(_event):
        try:
            tf = tf_buffer.lookup_transform(args.map_frame, args.base_frame, rospy.Time(0))
        except tf2_ros.TransformException:
            hub.console.clear_pose()
            return
        t, r = tf.transform.translation, tf.transform.rotation
        hub.console.set_pose(t.x, t.y, yaw_from_quaternion(r.x, r.y, r.z, r.w))

    cmd_pub = rospy.Publisher(args.cmd_vel_topic, Twist, queue_size=10)
    neupan_stop_pub = rospy.Publisher("/neupan/stop", Empty, queue_size=10)
    estop_pub = rospy.Publisher("/emergency_stop", Bool, queue_size=10)

    rospy.Subscriber(args.map_topic, OccupancyGrid, on_map, queue_size=1)
    rospy.Subscriber(args.odom_topic, Odometry, on_odom, queue_size=10)
    rospy.Subscriber(args.cmd_vel_topic, Twist, lambda m: hub.on_cmd_vel(m.linear.x, m.linear.y, m.angular.z),
                     queue_size=10)
    rospy.Subscriber("/waypoint_manager/system_health", String,
                     lambda m: hub.update_system_health_json(m.data), queue_size=1)
    if MissionStatus is not None:
        rospy.Subscriber("/waypoint_manager/status", MissionStatus, on_status, queue_size=10)
    if MissionEvent is not None:
        rospy.Subscriber("/waypoint_manager/mission_feedback", MissionEvent, on_event, queue_size=10)
    if args.scan_topic:
        rospy.Subscriber(args.scan_topic, LaserScan, on_scan, queue_size=1)
    rospy.Timer(rospy.Duration(1.0 / args.pose_hz), on_pose_tick)
    rospy.Timer(rospy.Duration(0.04), lambda _e: hub.nav.tick())

    def publish_cmd_vel(vx, vy, vyaw):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = float(vx), float(vy), float(vyaw)
        cmd_pub.publish(msg)

    def call_trigger(name):
        try:
            rospy.wait_for_service(name, timeout=2.0)
            res = rospy.ServiceProxy(name, Trigger)()
            return bool(res.success), str(res.message)
        except rospy.ROSException:
            return False, "服务 {} 当前未就绪（导航栈可能未启动）".format(name)
        except rospy.ServiceException as exc:
            return False, str(exc)

    def start_mission(points, loop, temporary):
        if StartMission is None:
            return False, "nav_interfaces 未安装，无法下发任务", ""
        waypoints = []
        for p in points:
            waypoint = Waypoint(x=p["x"], y=p["y"], yaw=p["yaw"], dwell_sec=p["dwell_sec"], name=p["name"])
            if hasattr(waypoint, "floor"):
                waypoint.floor = str(p.get("floor") or "")
            waypoints.append(waypoint)
        try:
            rospy.wait_for_service(START_MISSION_SERVICE, timeout=5.0)
            res = rospy.ServiceProxy(START_MISSION_SERVICE, StartMission)(
                waypoints=waypoints, loop=bool(loop), temporary=bool(temporary))
            return bool(res.success), str(res.message), str(res.mission_id)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            return False, str(exc), ""

    def _finite_ranges(msg):
        low, high = max(float(msg.range_min), 0.0), float(msg.range_max)
        return [float(value) for value in msg.ranges if low <= value <= high]

    def _front_range(msg, finite):
        """Narrowest return inside the forward sector (index-based, so lidar mounting-agnostic)."""
        if not finite:
            return None
        count = len(msg.ranges)
        width = max(1, int(round(FRONT_SECTOR_RAD / max(float(msg.angle_increment), 1e-6))))
        half = min(width // 2, count // 2)
        # The scan's first beam is treated as "forward"; two-sided window covers wraparound.
        window = list(msg.ranges[:half]) + list(msg.ranges[count - half:]) if half else []
        values = [float(value) for value in window
                  if max(float(msg.range_min), 0.0) <= value <= float(msg.range_max)]
        return min(values) if values else None

    def sample_scan(samples=3, interval_sec=0.2):
        """Averages finite lidar returns over a few consecutive scans."""
        count = max(1, min(int(samples), MAX_SCAN_SAMPLES))
        interval = max(0.0, min(float(interval_sec), 2.0))
        readings, fronts = [], []
        latest = None
        for index in range(count):
            with scan_lock:
                latest = scan_state["latest"]
            if latest is not None:
                finite = _finite_ranges(latest)
                if finite:
                    readings.append(finite)
                    front = _front_range(latest, finite)
                    if front is not None:
                        fronts.append(front)
            if index + 1 < count:
                time.sleep(interval)
        if not readings:
            return None
        flat = [value for reading in readings for value in reading]
        metrics = {
            "samples": len(readings),
            "points": len(flat),
            "min_range_m": round(min(flat), 3),
            "mean_range_m": round(sum(flat) / len(flat), 3),
            "max_range_m": round(max(flat), 3),
            "frame_id": latest.header.frame_id,
            "angle_min_rad": round(float(latest.angle_min), 4),
            "angle_max_rad": round(float(latest.angle_max), 4),
            "stamp": round(latest.header.stamp.to_sec(), 3),
        }
        if fronts:
            metrics["front_min_range_m"] = round(min(fronts), 3)
        return metrics

    def subscribe_image():
        with img_lock:
            if img_sub["handle"] is None:
                img_sub["handle"] = rospy.Subscriber(
                    args.image_topic, CompressedImage,
                    lambda m: hub.camera.on_compressed_frame(bytes(m.data)), queue_size=1)

    def unsubscribe_image():
        with img_lock:
            if img_sub["handle"] is not None:
                img_sub["handle"].unregister()
                img_sub["handle"] = None

    def list_topics():
        return {name: [typ] for name, typ in rospy.get_published_topics()}

    hub.bind(
        list_topics=list_topics,
        call_trigger=call_trigger,
        publish_cmd_vel=publish_cmd_vel,
        publish_neupan_stop=lambda: neupan_stop_pub.publish(Empty()),
        publish_estop=lambda active: estop_pub.publish(Bool(data=bool(active))),
        subscribe_image=subscribe_image,
        unsubscribe_image=unsubscribe_image,
        start_mission=start_mission,
        sample_scan=sample_scan,
    )
    rospy.spin()

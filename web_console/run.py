#!/usr/bin/env python3
"""Start the ONS2 web console. Source the ROS 1 or ROS 2 environment first."""

import argparse
import os
import threading
from dataclasses import replace

from web_console.hub import REPO_ROOT, MonitorStateHub
from web_console.inspection import InspectionConfig
from web_console.safety import StallDetector
from web_console.server import make_server


def parse_args():
    parser = argparse.ArgumentParser(description="ONS2 web console: map, control, safety, camera and intercom")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=18090, help="HTTP + HTTPS share this port")
    parser.add_argument("--no-tls", action="store_true", help="Disable the HTTPS side of the port")
    parser.add_argument("--map-topic", default="/map")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--pose-hz", type=float, default=10.0)
    parser.add_argument("--odom-topic", default="/odin1/odometry")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--image-topic", default="/odin1/image/compressed")
    parser.add_argument("--scan-topic", default="/scan_360",
                        help="LaserScan topic sampled by inspection sensor probes")
    parser.add_argument("--stall-window-sec", type=float, default=30.0)
    parser.add_argument("--stall-radius-m", type=float, default=0.20)
    parser.add_argument("--ros-version", choices=["1", "2"], default=os.environ.get("ROS_VERSION"))
    # Inspection task module (see docs/inspection.md). Enabled unless --no-inspection.
    parser.add_argument("--inspection", dest="inspection", action="store_true", default=None,
                        help="Accept structured inspection tasks from the backend (default)")
    parser.add_argument("--no-inspection", dest="inspection", action="store_false", default=None,
                        help="Disable the inspection task module")
    parser.add_argument("--inspection-config", default=os.environ.get("ONS2_INSPECTION_CONFIG"),
                        help="YAML file with an 'inspection:' section overriding the defaults")
    parser.add_argument("--inspection-dir", dest="inspection_storage_dir", default=None,
                        help="Task/artifact storage root (default ~/ons2_inspection)")
    parser.add_argument("--inspection-points", dest="inspection_points", default=None,
                        help="Point library YAML (default <repo>/inspection_points.yaml)")
    parser.add_argument("--inspection-token", dest="inspection_token", default=None,
                        help="Shared token required on /api/inspection requests")
    parser.add_argument("--inspection-callback", dest="inspection_callback", default=None,
                        help="Default callback URL for task reports")
    parser.add_argument("--inspection-no-callback", dest="inspection_callbacks",
                        action="store_false", default=None,
                        help="Keep reports local instead of POSTing them to the backend")
    parser.add_argument("--inspection-callback-host", dest="inspection_callback_hosts",
                        action="append", default=None,
                        help="Allowlist host for report callbacks (repeatable)")
    return parser.parse_args()


def _ensure_fastdds_large_shm():
    if os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE"):
        return
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    candidate = os.path.join(repo_root, "ros2_ws", "src", "map_planner", "config", "odin_cloud_filter_fastdds.xml")
    if os.path.isfile(candidate):
        os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = candidate


def build_inspection_config(args):
    try:
        config = InspectionConfig.resolve(args)
    except ValueError as exc:
        raise SystemExit("inspection configuration error: {}".format(exc))
    if not config.points_file:
        config = replace(config, points_file=os.path.join(REPO_ROOT, "inspection_points.yaml"))
    return config


def main():
    args = parse_args()
    ros_version = args.ros_version or "2"
    if ros_version == "1":
        from web_console import ros1_adapter as adapter
    else:
        _ensure_fastdds_large_shm()
        from web_console import ros2_adapter as adapter
    inspection_config = build_inspection_config(args)
    hub = MonitorStateHub(stall_detector=StallDetector(window_sec=args.stall_window_sec,
                                                       stall_radius_m=args.stall_radius_m),
                          inspection_config=inspection_config)
    server = make_server(args.host, args.port, hub, enable_tls=not args.no_tls)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    scheme = "http" if args.no_tls else "http/https"
    print("ONS2 web console: {}://{}:{} (ROS {})".format(scheme, args.host, args.port, ros_version), flush=True)
    if inspection_config.enabled:
        points = hub.inspection.status()["points"]
        print("Inspection module: enabled, storage={}, points={} ({} loaded{})".format(
            inspection_config.storage_path(), inspection_config.points_path() or "<none>",
            points["count"], "" if points["loaded"] else ", file missing"), flush=True)
        if points.get("error"):
            print("Inspection point library error: {}".format(points["error"]), flush=True)
    else:
        print("Inspection module: disabled", flush=True)
    try:
        adapter.run(args, hub)
    except KeyboardInterrupt:
        pass
    finally:
        hub.inspection.stop()
        server.shutdown()
        server.server_close()


if __name__ == "__main__":
    main()

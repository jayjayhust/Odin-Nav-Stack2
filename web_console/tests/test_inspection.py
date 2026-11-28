"""Tests for the inspection task module: schema, points, store, executor and HTTP API."""

import json
import threading
import urllib.error
import urllib.request

import pytest

from web_console.hub import MonitorStateHub
from web_console.inspection import InspectionConfig, InspectionManager, TaskSchemaError
from web_console.inspection.points import PointLibrary
from web_console.inspection.robot import RobotLinkStub
from web_console.inspection.schema import parse_task
from web_console.inspection.store import InspectionStore
from web_console.server import make_server

TOKEN = "s3cret"


# --------------------------------------------------------------------------- helpers
def task_payload(**overrides):
    payload = {
        "task_id": "t-1",
        "name": "night patrol",
        "steps": [{
            "step_id": "s1",
            "target": {"x": 1.0, "y": 2.0, "yaw": 0.5},
            "sensors": ["camera", "lidar_scan"],
            "actions": [{"type": "speak", "params": {"text": "inspection started"}}],
        }],
    }
    payload.update(overrides)
    return payload


class RecordingReporter:
    def __init__(self):
        self.sent = []
        self.started = False

    def start(self):
        self.started = True

    def stop(self):
        self.started = False

    def submit(self, url, payload):
        if url:
            self.sent.append((url, json.loads(json.dumps(payload))))

    def status(self):
        return {"enabled": True, "pending": 0}


def make_manager(tmp_path, robot=None, **settings):
    """Builds an executor whose worker thread is parked so each test drives `_tick()`."""
    options = {key: settings.pop(key) for key in ("clock", "sleep") if key in settings}
    config = dict(enabled=True, storage_dir=str(tmp_path), callbacks_enabled=False)
    config.update(settings)
    robot = robot or RobotLinkStub()
    if not robot.frames:
        robot.frames.append(b"jpeg-bytes")
    if not robot.scans:
        robot.scans.append({"samples": 3, "min_distance_m": 0.8})
    reporter = RecordingReporter()
    manager = InspectionManager(config=InspectionConfig(**config), robot=robot, reporter=reporter,
                                store=InspectionStore(str(tmp_path)), **options)
    manager._stop_worker()
    return manager, robot, reporter


def api_get(base, path, token=TOKEN):
    headers = {"X-Inspection-Token": token} if token else {}
    request = urllib.request.Request(base + path, headers=headers)
    try:
        with urllib.request.urlopen(request, timeout=5) as response:
            return response.status, response.headers, response.read()
    except urllib.error.HTTPError as exc:
        return exc.code, exc.headers, exc.read()


def api_post(base, path, payload, token=TOKEN):
    headers = {"Content-Type": "application/json"}
    if token:
        headers["X-Inspection-Token"] = token
    request = urllib.request.Request(base + path, data=json.dumps(payload).encode("utf-8"),
                                     method="POST", headers=headers)
    try:
        with urllib.request.urlopen(request, timeout=5) as response:
            return response.status, json.loads(response.read())
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read())


# ---------------------------------------------------------------------------- schema
def test_schema_normalizes_explicit_target_and_actions():
    task = parse_task(task_payload(), None, now=1_000.0)
    assert task.task_id == "t-1"
    assert task.steps[0].waypoint() == {"x": 1.0, "y": 2.0, "yaw": 0.5, "dwell_sec": 0.0,
                                        "name": "s1", "floor": ""}
    assert task.steps[0].sensors == ["camera", "lidar_scan"]
    assert task.steps[0].actions == [{"type": "speak",
                                      "params": {"text": "inspection started", "volume": 100}}]


@pytest.mark.parametrize("mutation", [
    {"unexpected": True},
    {"steps": []},
    {"steps": [{"target": {"x": 0, "y": 0}, "sensors": ["thermal"]}]},
    {"steps": [{"target": {"x": 0, "y": 0}, "actions": [{"type": "explode"}]}]},
    {"steps": [{"target": {"x": 0, "y": 0}, "actions": [{"type": "wait", "params": {"sec": 900}}]}]},
    {"steps": [{"target": {"x": 0}}]},
    {"steps": [{"target": {"x": 1, "y": 1, "point_id": "door"}}]},
    {"on_step_failure": "ignore"},
    {"callback_url": "ftp://example.invalid/report"},
])
def test_schema_rejects_invalid_payloads(mutation):
    with pytest.raises(TaskSchemaError):
        parse_task(task_payload(**mutation), None, now=1_000.0)


def test_schema_rejects_duplicate_step_ids():
    payload = task_payload(steps=[{"step_id": "same", "target": {"x": 0, "y": 0}},
                                  {"step_id": "same", "target": {"x": 1, "y": 1}}])
    with pytest.raises(TaskSchemaError):
        parse_task(payload, None, now=1_000.0)


def test_schema_resolves_point_ids_and_keeps_library_yaw():
    library = PointLibrary([{"id": "door", "name": "Door", "map": "1F", "floor": "1F",
                             "x": 4.0, "y": 5.0, "yaw": 1.25}])
    resolved = parse_task(task_payload(steps=[{"step_id": "s1", "target": {"point_id": "door"}}]),
                          library, now=1_000.0)
    assert (resolved.steps[0].x, resolved.steps[0].y, resolved.steps[0].yaw) == (4.0, 5.0, 1.25)
    assert resolved.steps[0].floor == "1F" and resolved.map == "1F"

    overridden = parse_task(task_payload(steps=[{"step_id": "s1",
                                                 "target": {"point_id": "door", "yaw": -0.5}}]),
                            library, now=1_000.0)
    assert overridden.steps[0].yaw == -0.5


def test_schema_requires_point_library_for_point_ids():
    with pytest.raises(TaskSchemaError):
        parse_task(task_payload(steps=[{"step_id": "s1", "target": {"point_id": "door"}}]),
                   None, now=1_000.0)


def test_schema_scheduling_window_is_validated():
    scheduled = parse_task(task_payload(start_at="2026-11-24T02:00:00Z",
                                        deadline_at="2026-11-24T03:00:00Z"), None,
                           now=1_767_000_000.0)
    assert scheduled.start_at is not None and scheduled.deadline_at > scheduled.start_at
    with pytest.raises(TaskSchemaError):
        parse_task(task_payload(start_at="2026-11-24T03:00:00Z",
                                deadline_at="2026-11-24T02:00:00Z"), None, now=1_767_000_000.0)

    # Local time without an offset is read as UTC, so it never depends on the robot's clock.
    assert parse_task(task_payload(start_at="2026-11-24 02:00:00"), None,
                      now=1_767_000_000.0).start_at == 1_795_485_600.0


def test_schema_generates_unique_ids():
    first = parse_task(task_payload(task_id=""), None, now=1_000.0)
    second = parse_task(task_payload(task_id=""), None, now=1_000.0)
    assert first.task_id != second.task_id


# ---------------------------------------------------------------------------- points
def test_point_library_loads_mapping_form(tmp_path):
    path = tmp_path / "points.yaml"
    path.write_text("schema_version: 1\npoints:\n  lobby:\n    x: 1.5\n    y: -2.0\n    yaw: 0.3\n"
                    "    map: 1F\n", encoding="utf-8")
    library = PointLibrary.load(str(path))
    assert library.status()["count"] == 1
    assert library.resolve("lobby", "")["yaw"] == 0.3
    assert library.list()[0]["id"] == "lobby"


def test_point_library_requires_map_when_ambiguous():
    library = PointLibrary([{"id": "door", "map": "1F", "x": 0, "y": 0, "yaw": 0.0},
                            {"id": "door", "map": "2F", "x": 9, "y": 9, "yaw": 0.0}])
    with pytest.raises(TaskSchemaError):
        library.resolve("door", "")
    assert library.resolve("door", "2F")["x"] == 9.0


# ----------------------------------------------------------------------------- store
def test_store_rejects_path_traversal(tmp_path):
    store = InspectionStore(str(tmp_path))
    with pytest.raises(ValueError):
        store.artifact_path("t-1", "../escape.jpg")
    with pytest.raises(ValueError):
        store.task_dir("../../etc")


def test_store_round_trips_state_artifact_and_report(tmp_path):
    store = InspectionStore(str(tmp_path))
    store.save_state({"task_id": "t-1", "state": "QUEUED"})
    assert store.load_state("t-1")["state"] == "QUEUED"
    assert [item["task_id"] for item in store.load_states()] == ["t-1"]

    saved = store.save_artifact("t-1", "frame.jpg", b"jpeg")
    assert saved["size_bytes"] == 4 and (tmp_path / "tasks" / "t-1" / "artifacts" / "frame.jpg").is_file()

    store.append_report("t-1", {"event": "ACCEPTED"})
    store.append_report("t-1", {"event": "FINISHED"})
    assert [entry["event"] for entry in store.read_report("t-1")] == ["ACCEPTED", "FINISHED"]
    assert store.read_report("t-1", limit=1) == [{"event": "FINISHED"}]


def test_store_prunes_oldest_finished_tasks(tmp_path):
    store = InspectionStore(str(tmp_path))
    for index in range(4):
        store.save_state({"task_id": "t-{}".format(index), "state": "SUCCEEDED",
                          "created_at": float(index)})
    store.save_state({"task_id": "keep", "state": "QUEUED", "created_at": 0.0})
    store.prune(keep=2)
    remaining = sorted(item["task_id"] for item in store.load_states())
    assert remaining == ["keep", "t-2", "t-3"]


# --------------------------------------------------------------------------- executor
def test_manager_runs_task_to_success(tmp_path):
    manager, robot, _ = make_manager(tmp_path)
    result = manager.submit(task_payload())
    assert result["accepted"][0]["task_id"] == "t-1" and result["rejected"] == []

    manager._tick()

    record = manager.get_task("t-1")
    assert record["state"] == "SUCCEEDED"
    assert record["progress"] == {"steps_total": 1, "steps_done": 1, "steps_failed": 0,
                                  "steps_skipped": 0, "current_step": 1}
    assert record["steps"][0]["state"] == "DONE"
    assert [entry["state"] for entry in record["steps"][0]["sensors"]] == ["OK", "OK"]
    assert record["steps"][0]["actions"][0]["state"] == "OK"
    assert robot.spoken == [("inspection started", 100)]

    # One single-waypoint mission per step, floor preserved for multi-floor stacks.
    assert len(robot.started) == 1
    assert robot.started[0] == [{"x": 1.0, "y": 2.0, "yaw": 0.5, "dwell_sec": 0.0,
                                 "name": "s1", "floor": ""}]

    artifacts = record["artifacts"]
    assert {item["name"].rsplit(".", 1)[-1] for item in artifacts} == {"jpg", "json"}
    assert all(item["size_bytes"] > 0 for item in artifacts)
    assert [entry["artifacts"][0]["kind"] for entry in record["steps"][0]["sensors"]] == \
        ["image", "scan"]
    events = [entry["event"] for entry in manager.report("t-1")]
    assert events[0] == "ACCEPTED" and events[-1] == "FINISHED"


def test_manager_duplicate_submit_is_idempotent(tmp_path):
    manager, robot, _ = make_manager(tmp_path)
    manager.submit(task_payload())
    again = manager.submit(task_payload())
    assert again["accepted"][0]["duplicate"] is True
    manager._tick()
    assert len(robot.started) == 1


def test_manager_rejects_unknown_fields_with_index(tmp_path):
    manager, _, _ = make_manager(tmp_path)
    result = manager.submit({"tasks": [task_payload(), task_payload(task_id="t-2", bogus=1)]})
    assert [item["task_id"] for item in result["accepted"]] == ["t-1"]
    assert result["rejected"][0]["index"] == 1


def test_manager_rejects_new_tasks_when_queue_is_full(tmp_path):
    manager, _, _ = make_manager(tmp_path, max_queue=1)
    manager.submit(task_payload())
    result = manager.submit(task_payload(task_id="t-2"))
    assert result["accepted"] == [] and "queue is full" in result["rejected"][0]["error"]


def test_manager_cancel_before_start(tmp_path):
    manager, robot, _ = make_manager(tmp_path)
    manager.submit(task_payload())
    assert manager.cancel("t-1")["state"] == "CANCELLED"
    manager._tick()
    assert robot.started == []
    assert manager.get_task("t-1")["state"] == "CANCELLED"
    with pytest.raises(KeyError):
        manager.cancel("nope")


def test_manager_promotes_scheduled_task_when_due(tmp_path):
    now = [1_000.0]
    manager, robot, _ = make_manager(tmp_path, clock=lambda: now[0], sleep=lambda sec: None)
    manager.submit(task_payload(start_at="1970-01-01T00:20:00Z"))  # epoch 1200
    assert manager.get_task("t-1")["state"] == "SCHEDULED"

    manager._tick()
    assert robot.started == [] and manager.get_task("t-1")["state"] == "SCHEDULED"

    now[0] = 1_201.0
    manager._tick()
    assert len(robot.started) == 1
    assert manager.get_task("t-1")["state"] == "SUCCEEDED"


def test_manager_fails_task_when_deadline_expires(tmp_path):
    now = [1_000.0]
    manager, robot, _ = make_manager(tmp_path, clock=lambda: now[0], sleep=lambda sec: None)
    manager.submit(task_payload(start_at="1970-01-01T00:20:00Z", deadline_at="1970-01-01T00:21:00Z"))
    now[0] = 1_300.0
    manager._tick()
    record = manager.get_task("t-1")
    assert record["state"] == "FAILED" and "deadline" in record["message"]
    assert robot.started == []


def test_manager_marks_step_partial_when_capture_fails(tmp_path):
    manager, robot, _ = make_manager(tmp_path)
    robot.frames.clear()
    manager.submit(task_payload(steps=[{"step_id": "s1", "target": {"x": 0, "y": 0},
                                        "actions": [{"type": "capture_image"}]}]))
    manager._tick()
    record = manager.get_task("t-1")
    # Nothing was accomplished, so the task fails while the step records what broke.
    assert record["state"] == "FAILED"
    assert record["steps"][0]["state"] == "PARTIAL"
    assert "no camera frame" in record["steps"][0]["message"]
    assert record["progress"]["steps_failed"] == 1


def test_manager_abort_stops_remaining_steps(tmp_path):
    robot = RobotLinkStub()
    robot.frames = [b"only-one-frame"]
    manager, _, _ = make_manager(tmp_path, robot=robot)
    steps = [{"step_id": "s{}".format(index), "target": {"x": index, "y": 0},
              "actions": [{"type": "capture_image"}]} for index in range(3)]
    manager.submit(task_payload(steps=steps, on_step_failure="abort"))

    manager._tick()

    record = manager.get_task("t-1")
    assert record["state"] == "PARTIAL"
    assert [step["state"] for step in record["steps"]] == ["DONE", "PARTIAL", "PENDING"]
    assert len(robot.started) == 2


def test_manager_retries_failed_navigation(tmp_path):
    robot = RobotLinkStub()
    robot.mission_state = "FAILED"
    manager, _, _ = make_manager(tmp_path, robot=robot, sleep=lambda sec: None)
    manager.submit(task_payload(steps=[{"step_id": "s1", "target": {"x": 0, "y": 0},
                                        "retries": 1, "timeout_sec": 30}]))
    manager._tick()
    record = manager.get_task("t-1")
    assert record["state"] == "FAILED"
    assert record["steps"][0]["attempts"] == 2
    assert len(robot.started) == 2 and robot.cancels == 2


def test_manager_blocks_operator_missions_while_running(tmp_path):
    manager, _, _ = make_manager(tmp_path)
    assert manager.operator_mission_blocked() == (False, "")
    manager._active_id = "t-1"
    blocked, reason = manager.operator_mission_blocked()
    assert blocked is True and "t-1" in reason


def test_manager_pushes_terminal_report_to_callback(tmp_path):
    manager, _, reporter = make_manager(tmp_path, default_callback_url="http://backend.invalid/hook")
    manager.submit(task_payload())
    manager._tick()
    events = [payload["event"] for _, payload in reporter.sent]
    assert "FINISHED" in events
    finished = [payload for _, payload in reporter.sent if payload["event"] == "FINISHED"][-1]
    assert finished["state"] == "SUCCEEDED" and finished["artifacts"]


def test_manager_replays_running_task_as_interrupted(tmp_path):
    store = InspectionStore(str(tmp_path))
    store.save_state({"task_id": "t-1", "state": "RUNNING", "created_at": 1.0, "steps": [],
                      "progress": {}, "name": "x", "priority": 0, "map": "", "run_index": 1})
    manager, _, _ = make_manager(tmp_path)
    record = manager.get_task("t-1")
    assert record["state"] == "INTERRUPTED"
    assert store.load_state("t-1")["state"] == "INTERRUPTED"


def test_manager_survives_a_broken_point_library(tmp_path):
    bad = tmp_path / "points.yaml"
    bad.write_text("schema_version: 1\npoints: {door: {x: 1}}\n", encoding="utf-8")
    manager, _, _ = make_manager(tmp_path, points_file=str(bad))
    points = manager.status()["points"]
    assert points["error"]
    assert points["count"] == 0
    assert manager.status()["enabled"] is True


# ------------------------------------------------------------------------------- API
@pytest.fixture
def served(tmp_path):
    config = InspectionConfig(enabled=True, storage_dir=str(tmp_path), token=TOKEN)
    hub = MonitorStateHub(intercom=None, inspection_config=config)
    hub.inspection.robot = RobotLinkStub()
    server = make_server("127.0.0.1", 0, hub, enable_tls=False)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        yield hub, "http://127.0.0.1:{}".format(server.server_address[1])
    finally:
        server.shutdown()
        server.server_close()
        hub.inspection.stop()


def test_api_requires_token_when_configured(served):
    _, base = served
    status, _, _ = api_get(base, "/api/inspection/status", token="")
    assert status == 401
    status, _, body = api_get(base, "/api/inspection/status", token="wrong")
    assert status == 401
    status, _, body = api_get(base, "/api/inspection/status")
    assert status == 200 and json.loads(body)["enabled"] is True


def test_api_exposes_schema_and_points(served):
    _, base = served
    status, _, body = api_get(base, "/api/inspection/schema")
    payload = json.loads(body)
    assert status == 200 and "measure_scan" in payload["actions"]
    status, _, body = api_get(base, "/api/inspection/points")
    assert status == 200 and json.loads(body)["points"] == []


def test_api_submits_fetches_and_cancels_task(served):
    _, base = served
    payload = task_payload(start_at="2035-01-01T00:00:00Z")
    status, created = api_post(base, "/api/inspection/tasks", payload)
    assert status == 200 and created["accepted"][0]["state"] == "SCHEDULED"

    status, _, body = api_get(base, "/api/inspection/tasks/t-1")
    assert status == 200 and json.loads(body)["state"] == "SCHEDULED"

    status, _, body = api_get(base, "/api/inspection/tasks")
    assert [item["task_id"] for item in json.loads(body)["tasks"]] == ["t-1"]

    status, _, body = api_get(base, "/api/inspection/tasks/t-1/report")
    assert [entry["event"] for entry in json.loads(body)["entries"]] == ["ACCEPTED"]

    status, cancelled = api_post(base, "/api/inspection/tasks/t-1/cancel", {})
    assert status == 200 and cancelled["state"] == "CANCELLED"


def test_api_rejects_invalid_task_with_per_index_errors(served):
    _, base = served
    status, body = api_post(base, "/api/inspection/tasks", task_payload(bogus=1))
    assert status == 400 and body["ok"] is False
    assert body["rejected"][0]["index"] == 0


def test_api_serves_task_artifacts(served):
    hub, base = served
    api_post(base, "/api/inspection/tasks", task_payload(start_at="2035-01-01T00:00:00Z"))
    hub.inspection.store.save_artifact("t-1", "frame.jpg", b"jpeg-bytes")

    status, _, body = api_get(base, "/api/inspection/tasks/t-1/artifacts")
    artifacts = json.loads(body)["artifacts"]
    assert [item["name"] for item in artifacts] == ["frame.jpg"]

    status, headers, body = api_get(base, "/api/inspection/tasks/t-1/artifacts/frame.jpg")
    assert status == 200 and body == b"jpeg-bytes" and headers["Content-Type"] == "image/jpeg"


def test_api_unknown_task_and_path_return_404(served):
    _, base = served
    status, _, _ = api_get(base, "/api/inspection/tasks/nope")
    assert status == 404
    status, _, _ = api_get(base, "/api/inspection/nonsense")
    assert status == 404


def test_hub_snapshot_carries_inspection_state():
    hub = MonitorStateHub(intercom=None)
    try:
        assert hub.get_snapshot()["inspection"]["enabled"] is False
        assert hub.inspection.points.status()["count"] == 0
    finally:
        hub.inspection.stop()

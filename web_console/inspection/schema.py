"""Structured inspection task schema: strict validation and normalization.

The inspection backend sends JSON tasks; this module is the single place that
decides whether a task is acceptable. It is deliberately ROS-agnostic so it can
be tested (and reused by a future standalone agent) without a robot.
"""

import math
import re
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Dict, List, Optional

SCHEMA_VERSION = 1
MAX_STEPS = 64
MAX_ACTIONS_PER_STEP = 16
MAX_BURST_FRAMES = 30
MAX_TEXT_CHARS = 200
MAX_WAIT_SEC = 600.0
MAX_STEP_TIMEOUT_SEC = 3600.0
MAX_TASK_HOURS = 12.0

TASK_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:-]{0,63}$")

# Task lifecycle. Terminal states never transition again.
TASK_STATES = (
    "QUEUED",       # accepted, waiting for its turn
    "SCHEDULED",    # accepted, start_at is still in the future
    "RUNNING",      # owns the robot right now
    "SUCCEEDED",    # every step done
    "PARTIAL",      # finished, at least one step failed/skipped
    "FAILED",       # could not run (aborted, safety, dispatch rejected)
    "CANCELLED",    # cancelled by the backend or an operator
    "INTERRUPTED",  # console restarted while the task was running
)
TERMINAL_TASK_STATES = frozenset(
    {"SUCCEEDED", "PARTIAL", "FAILED", "CANCELLED", "INTERRUPTED"})

# Per-step lifecycle.
STEP_STATES = ("PENDING", "NAVIGATING", "ARRIVED", "ACTING", "DONE", "PARTIAL",
               "FAILED", "SKIPPED", "CANCELLED")
STEP_FAILED_STATES = frozenset({"PARTIAL", "FAILED", "SKIPPED"})

SENSOR_TYPES = ("camera", "lidar_scan")
ACTION_TYPES = ("capture_image", "capture_burst", "measure_scan", "speak", "wait")

_TASK_KEYS = {
    "schema_version", "task_id", "name", "priority", "map", "start_at", "deadline_at",
    "repeat", "on_step_failure", "callback_url", "steps", "metadata",
}
_STEP_KEYS = {
    "step_id", "name", "target", "timeout_sec", "retries", "dwell_sec",
    "sensors", "actions",
}
_TARGET_KEYS = {"point_id", "x", "y", "yaw", "floor"}
_REPEAT_KEYS = {"count", "interval_sec"}


class TaskSchemaError(ValueError):
    """Raised when a task payload is structurally invalid or self-inconsistent."""


_ID_LOCK = threading.Lock()
_ID_STATE = {"seq": 0}


def _fail(message: str) -> None:
    raise TaskSchemaError(message)


def _number(value, field_name: str) -> float:
    if isinstance(value, bool) or value is None:
        _fail("{} must be a number".format(field_name))
    try:
        result = float(value)
    except (TypeError, ValueError):
        _fail("{} must be a number".format(field_name))
    if not math.isfinite(result):
        _fail("{} must be finite".format(field_name))
    return result


def _int(value, field_name: str, minimum: int, maximum: int, default: int) -> int:
    if value is None:
        return default
    if isinstance(value, bool):
        _fail("{} must be an integer".format(field_name))
    try:
        result = int(value)
    except (TypeError, ValueError):
        _fail("{} must be an integer".format(field_name))
    if result < minimum or result > maximum:
        _fail("{} must be between {} and {}".format(field_name, minimum, maximum))
    return result


def _seconds(value, field_name: str, minimum: float, maximum: float, default: float) -> float:
    if value is None:
        return default
    result = _number(value, field_name)
    if result < minimum or result > maximum:
        _fail("{} must be between {:g} and {:g}".format(field_name, minimum, maximum))
    return result


def parse_time(value, field_name: str) -> Optional[float]:
    """Parses ISO-8601 / epoch seconds into a POSIX timestamp (UTC)."""
    if value is None or (isinstance(value, str) and not value.strip()):
        return None
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        result = float(value)
        if not math.isfinite(result):
            _fail("{} must be finite".format(field_name))
        return result
    text = str(value).strip().replace("Z", "+00:00")
    parsed = None
    try:
        parsed = datetime.fromisoformat(text)
    except ValueError:
        for fmt in ("%Y-%m-%d %H:%M:%S", "%Y-%m-%d %H:%M", "%Y-%m-%d"):
            try:
                parsed = datetime.strptime(text, fmt)
                break
            except ValueError:
                continue
    if parsed is None:
        _fail("{} must be ISO-8601 (e.g. 2026-11-23T21:00:00+08:00) or epoch seconds".format(field_name))
    if parsed.tzinfo is None:
        parsed = parsed.replace(tzinfo=timezone.utc)
    return parsed.timestamp()


def format_time(timestamp: Optional[float]) -> Optional[str]:
    if timestamp is None:
        return None
    return datetime.fromtimestamp(float(timestamp), tz=timezone.utc).isoformat()


def _check_keys(raw: Dict, allowed, field_name: str) -> None:
    unknown = sorted(set(raw) - set(allowed))
    if unknown:
        _fail("{} has unknown field(s): {}".format(field_name, ", ".join(unknown)))


def _normalize_action(raw, index: int) -> Dict:
    where = "actions[{}]".format(index)
    if not isinstance(raw, dict):
        _fail("{} must be an object".format(where))
    action_type = str(raw.get("type") or "").strip()
    if action_type not in ACTION_TYPES:
        _fail("{} unsupported type '{}' (allowed: {})".format(
            where, action_type, ", ".join(ACTION_TYPES)))
    _check_keys(raw, {"type", "params"}, where)
    params = raw.get("params") or {}
    if not isinstance(params, dict):
        _fail("{}.params must be an object".format(where))

    if action_type == "capture_image":
        _check_keys(params, {"count"}, where + ".params")
        normalized = {"count": _int(params.get("count"), where + ".count", 1, MAX_BURST_FRAMES, 1)}
    elif action_type == "capture_burst":
        _check_keys(params, {"count", "interval_sec"}, where + ".params")
        normalized = {
            "count": _int(params.get("count"), where + ".count", 1, MAX_BURST_FRAMES, 5),
            "interval_sec": _seconds(params.get("interval_sec"), where + ".interval_sec", 0.0, 5.0, 0.5),
        }
    elif action_type == "measure_scan":
        _check_keys(params, {"samples", "interval_sec"}, where + ".params")
        normalized = {
            "samples": _int(params.get("samples"), where + ".samples", 1, 20, 3),
            "interval_sec": _seconds(params.get("interval_sec"), where + ".interval_sec", 0.0, 2.0, 0.2),
        }
    elif action_type == "speak":
        _check_keys(params, {"text", "volume"}, where + ".params")
        text = str(params.get("text") or "").strip()
        if not text:
            _fail("{}.text must be a non-empty string".format(where))
        if len(text) > MAX_TEXT_CHARS:
            _fail("{}.text must be at most {} characters".format(where, MAX_TEXT_CHARS))
        normalized = {
            "text": text,
            "volume": _int(params.get("volume"), where + ".volume", 0, 100, 100),
        }
    else:  # wait
        _check_keys(params, {"sec"}, where + ".params")
        normalized = {"sec": _seconds(params.get("sec"), where + ".sec", 0.0, MAX_WAIT_SEC, 1.0)}
    return {"type": action_type, "params": normalized}


def _normalize_target(raw, where: str) -> Dict:
    if not isinstance(raw, dict):
        _fail("{} must be an object with point_id or x/y".format(where))
    _check_keys(raw, _TARGET_KEYS, where)
    point_id = str(raw.get("point_id") or "").strip()
    has_pose = raw.get("x") is not None or raw.get("y") is not None
    if point_id and has_pose:
        _fail("{} must use either point_id or x/y, not both".format(where))
    if not point_id and not has_pose:
        _fail("{} needs a point_id or an x/y pair".format(where))
    target = {
        "point_id": point_id,
        "x": None,
        "y": None,
        "yaw": _number(raw.get("yaw", 0.0), where + ".yaw"),
        "floor": str(raw.get("floor") or "").strip(),
    }
    if has_pose:
        if raw.get("x") is None or raw.get("y") is None:
            _fail("{} needs both x and y".format(where))
        target["x"] = _number(raw["x"], where + ".x")
        target["y"] = _number(raw["y"], where + ".y")
    else:
        target["yaw_specified"] = raw.get("yaw") is not None
    return target


def _normalize_step(raw, index: int) -> "Step":
    where = "steps[{}]".format(index)
    if not isinstance(raw, dict):
        _fail("{} must be an object".format(where))
    _check_keys(raw, _STEP_KEYS, where)
    step_id = str(raw.get("step_id") or "").strip() or "s{}".format(index + 1)
    if not re.match(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,31}$", step_id):
        _fail("{}.step_id may only contain letters, digits, '.', '_' and '-'".format(where))
    target = _normalize_target(raw.get("target"), where + ".target")

    sensors = raw.get("sensors") or []
    if not isinstance(sensors, list):
        _fail("{}.sensors must be a list".format(where))
    normalized_sensors = []
    for sensor in sensors:
        name = str(sensor or "").strip()
        if name not in SENSOR_TYPES:
            _fail("{}.sensors has unsupported sensor '{}' (allowed: {})".format(
                where, name, ", ".join(SENSOR_TYPES)))
        if name not in normalized_sensors:
            normalized_sensors.append(name)

    raw_actions = raw.get("actions") or []
    if not isinstance(raw_actions, list):
        _fail("{}.actions must be a list".format(where))
    if len(raw_actions) > MAX_ACTIONS_PER_STEP:
        _fail("{}.actions supports at most {} entries".format(where, MAX_ACTIONS_PER_STEP))
    actions = [_normalize_action(item, position) for position, item in enumerate(raw_actions)]

    return Step(
        step_id=step_id,
        name=str(raw.get("name") or step_id),
        x=target["x"],
        y=target["y"],
        yaw=target["yaw"],
        floor=target["floor"],
        point_id=target["point_id"],
        yaw_specified=bool(target.get("yaw_specified")),
        timeout_sec=_seconds(raw.get("timeout_sec"), where + ".timeout_sec",
                             1.0, MAX_STEP_TIMEOUT_SEC, 180.0),
        retries=_int(raw.get("retries"), where + ".retries", 0, 3, 0),
        dwell_sec=_seconds(raw.get("dwell_sec"), where + ".dwell_sec", 0.0, 300.0, 0.0),
        sensors=normalized_sensors,
        actions=actions,
    )


@dataclass
class Step:
    """One waypoint plus the sensors/actions to run once the robot is parked there."""

    step_id: str
    name: str
    x: Optional[float]
    y: Optional[float]
    yaw: float
    floor: str = ""
    point_id: str = ""
    yaw_specified: bool = False
    timeout_sec: float = 180.0
    retries: int = 0
    dwell_sec: float = 0.0
    sensors: List[str] = field(default_factory=list)
    actions: List[Dict] = field(default_factory=list)

    def waypoint(self) -> Dict:
        if self.x is None or self.y is None:
            _fail("step '{}' has no resolved pose".format(self.step_id))
        return {
            "x": float(self.x),
            "y": float(self.y),
            "yaw": float(self.yaw),
            "dwell_sec": float(self.dwell_sec),
            "name": self.name,
            "floor": self.floor,
        }

    def to_dict(self) -> Dict:
        return {
            "step_id": self.step_id,
            "name": self.name,
            "point_id": self.point_id,
            "x": self.x,
            "y": self.y,
            "yaw": self.yaw,
            "floor": self.floor,
            "timeout_sec": self.timeout_sec,
            "retries": self.retries,
            "dwell_sec": self.dwell_sec,
            "sensors": list(self.sensors),
            "actions": [dict(action) for action in self.actions],
        }


@dataclass
class Task:
    """A validated, normalized inspection task."""

    task_id: str
    name: str
    steps: List[Step]
    priority: int = 0
    map: str = ""
    start_at: Optional[float] = None
    deadline_at: Optional[float] = None
    repeat_count: int = 1
    repeat_interval_sec: float = 0.0
    on_step_failure: str = "continue"
    callback_url: str = ""
    metadata: Dict = field(default_factory=dict)

    def to_dict(self) -> Dict:
        return {
            "schema_version": SCHEMA_VERSION,
            "task_id": self.task_id,
            "name": self.name,
            "priority": self.priority,
            "map": self.map,
            "start_at": format_time(self.start_at),
            "start_at_epoch": self.start_at,
            "deadline_at": format_time(self.deadline_at),
            "repeat": {"count": self.repeat_count, "interval_sec": self.repeat_interval_sec},
            "on_step_failure": self.on_step_failure,
            "callback_url": self.callback_url,
            "metadata": self.metadata,
            "steps": [step.to_dict() for step in self.steps],
        }


def new_task_id(now: Optional[float] = None) -> str:
    """Generates an id for tasks submitted without one (never collides within a batch)."""
    stamp = int((time.time() if now is None else now) * 1000.0)
    with _ID_LOCK:
        _ID_STATE["seq"] += 1
        seq = _ID_STATE["seq"]
    return "insp_{}_{:04d}".format(stamp, seq % 10000)


def parse_task(raw, points=None, now: Optional[float] = None) -> Task:
    """Validates a JSON task and resolves point ids through a point library.

    `points` is a `web_console.inspection.points.PointLibrary` (or None when the
    task uses explicit x/y targets only).
    """
    if not isinstance(raw, dict):
        _fail("task must be a JSON object")
    _check_keys(raw, _TASK_KEYS, "task")

    version = raw.get("schema_version", SCHEMA_VERSION)
    if version not in (SCHEMA_VERSION, str(SCHEMA_VERSION)):
        _fail("unsupported schema_version {} (this console accepts {})".format(version, SCHEMA_VERSION))

    task_id = str(raw.get("task_id") or "").strip()
    if task_id:
        if not TASK_ID_PATTERN.match(task_id):
            _fail("task_id must be 1-64 chars of letters, digits, '.', '_', ':' or '-'")
    else:
        task_id = new_task_id(now)

    raw_steps = raw.get("steps")
    if not isinstance(raw_steps, list) or not raw_steps:
        _fail("steps must be a non-empty list")
    if len(raw_steps) > MAX_STEPS:
        _fail("a task supports at most {} steps".format(MAX_STEPS))
    steps = [_normalize_step(item, index) for index, item in enumerate(raw_steps)]
    seen = set()
    for step in steps:
        if step.step_id in seen:
            _fail("duplicate step_id '{}'".format(step.step_id))
        seen.add(step.step_id)

    task_map = str(raw.get("map") or "").strip()
    for step in steps:
        if step.point_id:
            if points is None:
                _fail("step '{}' uses point_id '{}' but no point library is configured".format(
                    step.step_id, step.point_id))
            resolved = points.resolve(step.point_id, task_map)
            step.x, step.y = resolved["x"], resolved["y"]
            step.yaw = resolved["yaw"] if not step.yaw_specified else step.yaw
            if not step.floor:
                step.floor = resolved.get("floor", "")
            if not task_map and resolved.get("map"):
                task_map = resolved["map"]

    repeat = raw.get("repeat")
    repeat_count, repeat_interval = 1, 0.0
    if repeat is not None:
        if isinstance(repeat, int) and not isinstance(repeat, bool):
            repeat = {"count": repeat}
        if not isinstance(repeat, dict):
            _fail("repeat must be an object with count/interval_sec")
        _check_keys(repeat, _REPEAT_KEYS, "repeat")
        repeat_count = _int(repeat.get("count"), "repeat.count", 1, 1000, 1)
        repeat_interval = _seconds(repeat.get("interval_sec"), "repeat.interval_sec",
                                   0.0, 7 * 24 * 3600.0, 0.0)

    on_step_failure = str(raw.get("on_step_failure") or "continue").strip().lower()
    if on_step_failure not in ("continue", "abort"):
        _fail("on_step_failure must be 'continue' or 'abort'")

    callback_url = str(raw.get("callback_url") or "").strip()
    if callback_url and not (callback_url.startswith("http://") or callback_url.startswith("https://")):
        _fail("callback_url must start with http:// or https://")

    metadata = raw.get("metadata") or {}
    if not isinstance(metadata, dict):
        _fail("metadata must be an object")

    deadline_at = parse_time(raw.get("deadline_at"), "deadline_at")
    start_at = parse_time(raw.get("start_at"), "start_at")
    reference = time.time() if now is None else float(now)
    if deadline_at is not None and deadline_at <= reference:
        _fail("deadline_at is already in the past")
    if deadline_at is not None and start_at is not None and deadline_at <= start_at:
        _fail("deadline_at must be after start_at")

    return Task(
        task_id=task_id,
        name=str(raw.get("name") or task_id),
        steps=steps,
        priority=_int(raw.get("priority"), "priority", -1000, 1000, 0),
        map=task_map,
        start_at=start_at,
        deadline_at=deadline_at,
        repeat_count=repeat_count,
        repeat_interval_sec=repeat_interval,
        on_step_failure=on_step_failure,
        callback_url=callback_url,
        metadata=metadata,
    )


def describe_schema() -> Dict:
    """Machine-readable contract for the inspection backend (GET /api/inspection/schema)."""
    return {
        "schema_version": SCHEMA_VERSION,
        "task_states": list(TASK_STATES),
        "step_states": list(STEP_STATES),
        "limits": {
            "max_steps": MAX_STEPS,
            "max_actions_per_step": MAX_ACTIONS_PER_STEP,
            "max_step_timeout_sec": MAX_STEP_TIMEOUT_SEC,
            "max_wait_sec": MAX_WAIT_SEC,
        },
        "sensors": list(SENSOR_TYPES),
        "actions": {
            "capture_image": {"params": {"count": "1-{}".format(MAX_BURST_FRAMES)}},
            "capture_burst": {"params": {"count": "1-{}".format(MAX_BURST_FRAMES),
                                         "interval_sec": "0-5"}},
            "measure_scan": {"params": {"samples": "1-20", "interval_sec": "0-2"}},
            "speak": {"params": {"text": "1-{} chars".format(MAX_TEXT_CHARS), "volume": "0-100"}},
            "wait": {"params": {"sec": "0-{:g}".format(MAX_WAIT_SEC)}},
        },
        "target": {
            "either": "point_id (resolved from the point library)",
            "or": {"x": "map metres", "y": "map metres", "yaw": "radians, optional"},
            "floor": "logical floor id, optional (multi-floor missions)",
        },
        "time": "ISO-8601 with offset, or epoch seconds; naive values are treated as UTC",
    }

"""Inspection action and sensor handlers.

`step.sensors` is the declarative "what to measure here" list and always runs
first; `step.actions` is the ordered "what to do here" list. Both produce the
same result entries so the backend sees one uniform report.
"""

import json
import time
from typing import Dict, List

CAPTURE_ACTION_TIMEOUT_SEC = 3.0


class ActionError(RuntimeError):
    """Raised when an action cannot produce its result (camera dead, no scan...)."""


class Cancelled(Exception):
    """Raised inside long waits when the task was cancelled."""


class ActionContext:
    def __init__(self, robot, task, step, store, cancel_event):
        self.robot = robot
        self.task = task
        self.step = step
        self.store = store
        self.cancel_event = cancel_event
        self._counter = 0

    def check_cancel(self) -> None:
        if self.cancel_event is not None and self.cancel_event.is_set():
            raise Cancelled()

    def sleep(self, seconds: float) -> None:
        """Cancellable sleep in small slices so cancel is honoured quickly."""
        remaining = max(0.0, float(seconds))
        while remaining > 0.0:
            self.check_cancel()
            slice_sec = min(0.2, remaining)
            time.sleep(slice_sec)
            remaining -= slice_sec

    def unique_name(self, kind: str, extension: str) -> str:
        self._counter += 1
        stamp = int(time.time() * 1000)
        return "{}_{}_{}_{:02d}.{}".format(
            self.step.step_id, kind, stamp, self._counter % 100, extension)

    def save_artifact(self, name: str, data: bytes, kind: str) -> Dict:
        saved = self.store.save_artifact(self.task.task_id, name, data)
        saved["kind"] = kind
        return saved


def _capture_frames(ctx: ActionContext, count: int, interval_sec: float) -> Dict:
    artifacts: List[Dict] = []
    missing = 0
    for index in range(count):
        ctx.check_cancel()
        jpeg, stamp = ctx.robot.capture_jpeg(timeout=CAPTURE_ACTION_TIMEOUT_SEC)
        if not jpeg:
            missing += 1
        else:
            artifacts.append(ctx.save_artifact(ctx.unique_name("image", "jpg"), jpeg, "image"))
        if interval_sec > 0.0 and index + 1 < count:
            ctx.sleep(interval_sec)
    if not artifacts:
        raise ActionError("no camera frame arrived within {:g}s".format(CAPTURE_ACTION_TIMEOUT_SEC))
    return {
        "message": "captured {} frame(s)".format(len(artifacts)),
        "artifacts": artifacts,
        "metrics": {"requested": count, "captured": len(artifacts), "missing": missing,
                    "last_frame_stamp": round(float(stamp), 3) if artifacts else 0.0},
    }


def handle_capture_image(ctx: ActionContext, params: Dict) -> Dict:
    return _capture_frames(ctx, int(params.get("count", 1)), 0.0)


def handle_capture_burst(ctx: ActionContext, params: Dict) -> Dict:
    return _capture_frames(ctx, int(params.get("count", 5)), float(params.get("interval_sec", 0.5)))


def handle_measure_scan(ctx: ActionContext, params: Dict) -> Dict:
    metrics = ctx.robot.sample_scan(samples=int(params.get("samples", 3)),
                                    interval_sec=float(params.get("interval_sec", 0.2)))
    if not metrics:
        raise ActionError("no lidar scan data available on the configured scan topic")
    payload = json.dumps(metrics, ensure_ascii=False, sort_keys=True).encode("utf-8")
    artifact = ctx.save_artifact(ctx.unique_name("scan", "json"), payload, "scan")
    return {
        "message": "min range {:g} m".format(metrics.get("min_range_m") or 0.0),
        "artifacts": [artifact],
        "metrics": metrics,
    }


def handle_speak(ctx: ActionContext, params: Dict) -> Dict:
    ctx.check_cancel()
    ok, message = ctx.robot.speak(str(params["text"]), int(params.get("volume", 100)))
    if not ok:
        raise ActionError("speech failed: {}".format(message))
    return {"message": message,
            "metrics": {"text": params["text"], "volume": int(params.get("volume", 100))}}


def handle_wait(ctx: ActionContext, params: Dict) -> Dict:
    seconds = float(params.get("sec", 1.0))
    ctx.sleep(seconds)
    return {"message": "waited {:g}s".format(seconds), "metrics": {"sec": seconds}}


ACTIONS = {
    "capture_image": handle_capture_image,
    "capture_burst": handle_capture_burst,
    "measure_scan": handle_measure_scan,
    "speak": handle_speak,
    "wait": handle_wait,
}


def _sensor_camera(ctx: ActionContext) -> Dict:
    jpeg, stamp = ctx.robot.capture_jpeg(timeout=CAPTURE_ACTION_TIMEOUT_SEC)
    if not jpeg:
        raise ActionError("camera produced no frame within {:g}s".format(CAPTURE_ACTION_TIMEOUT_SEC))
    artifact = ctx.save_artifact(ctx.unique_name("sensor_camera", "jpg"), jpeg, "image")
    return {"message": "camera frame captured", "artifacts": [artifact],
            "metrics": {"frame_stamp": round(float(stamp), 3)}}


def _sensor_scan(ctx: ActionContext) -> Dict:
    metrics = ctx.robot.sample_scan(samples=3, interval_sec=0.2)
    if not metrics:
        raise ActionError("no lidar scan data available on the configured scan topic")
    payload = json.dumps(metrics, ensure_ascii=False, sort_keys=True).encode("utf-8")
    artifact = ctx.save_artifact(ctx.unique_name("sensor_scan", "json"), payload, "scan")
    return {"message": "scan sampled", "artifacts": [artifact], "metrics": metrics}


SENSOR_PROBES = {
    "camera": _sensor_camera,
    "lidar_scan": _sensor_scan,
}


def _entry(kind: str, name: str, params: Dict, state: str, message: str,
           started_at: float, artifacts: List[Dict], metrics: Dict) -> Dict:
    return {
        "kind": kind,
        "name": name,
        "params": params,
        "state": state,
        "message": message,
        "started_at": started_at,
        "finished_at": time.time(),
        "duration_sec": round(time.time() - started_at, 3),
        "artifacts": artifacts,
        "metrics": metrics,
    }


def run_sensor(ctx: ActionContext, sensor: str) -> Dict:
    """Runs one declarative sensor probe; failures are reported, not raised."""
    probe = SENSOR_PROBES.get(sensor)
    started_at = time.time()
    if probe is None:
        return _entry("sensor", sensor, {}, "FAILED", "unsupported sensor",
                      started_at, [], {})
    try:
        result = probe(ctx)
    except Cancelled:
        raise
    except Exception as exc:
        return _entry("sensor", sensor, {}, "FAILED", str(exc) or exc.__class__.__name__,
                      started_at, [], {})
    return _entry("sensor", sensor, {}, "OK", result.get("message", ""), started_at,
                  result.get("artifacts", []), result.get("metrics", {}))


def run_action(ctx: ActionContext, action: Dict) -> Dict:
    """Runs one step action; failures are reported, not raised (cancel still propagates)."""
    action_type = action.get("type", "")
    params = action.get("params") or {}
    handler = ACTIONS.get(action_type)
    started_at = time.time()
    if handler is None:
        return _entry("action", action_type, params, "FAILED", "unsupported action type",
                      started_at, [], {})
    try:
        result = handler(ctx, params)
    except Cancelled:
        raise
    except Exception as exc:
        return _entry("action", action_type, params, "FAILED", str(exc) or exc.__class__.__name__,
                      started_at, [], {})
    return _entry("action", action_type, params, "OK", result.get("message", ""), started_at,
                  result.get("artifacts", []), result.get("metrics", {}))


def describe_actions() -> Dict:
    return {
        "sensors": sorted(SENSOR_PROBES),
        "actions": sorted(ACTIONS),
    }

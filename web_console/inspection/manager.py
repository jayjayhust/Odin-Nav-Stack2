"""Inspection task scheduler and executor.

The manager is the module's whole public surface: it queues tasks dispatched by
the inspection backend, decides when each one may drive the robot, executes the
steps against `mission_manager` (one single-waypoint mission per step), runs the
sensor/action handlers, and reports progress locally and to the backend.

Design note: one mission per step means the robot is always parked while
sensors/actions run. That costs a replan per step but gives per-step retries,
skip-on-failure, floor-transit safety and no race between dwell timers and
action durations.
"""

import copy
import hmac
import os
import threading
import time
from typing import Dict, List, Optional, Tuple

from web_console.inspection.actions import (ActionContext, Cancelled, describe_actions,
                                            run_action, run_sensor)
from web_console.inspection.config import InspectionConfig
from web_console.inspection.points import PointLibrary
from web_console.inspection.reporter import TaskReporter
from web_console.inspection.robot import RobotLink, RobotLinkStub
from web_console.inspection.schema import (SCHEMA_VERSION, TASK_STATES,
                                           TERMINAL_TASK_STATES, Task, TaskSchemaError,
                                           describe_schema, format_time, parse_task)
from web_console.inspection.store import InspectionStore

TICK_SEC = 0.2
MISSION_POLL_SEC = 0.2
RETRY_BACKOFF_SEC = 1.5
STALL_GRACE_SEC = 3.0
MAX_SUBMIT_BATCH = 50
MAX_MESSAGE_CHARS = 400


def _iso(timestamp: Optional[float]) -> Optional[str]:
    return format_time(timestamp)


def _new_step_record(step) -> Dict:
    return {
        "step_id": step.step_id,
        "name": step.name,
        "point_id": step.point_id,
        "state": "PENDING",
        "message": "",
        "attempts": 0,
        "arrived_at": None,
        "pose": None,
        "sensors": [],
        "actions": [],
    }


def _new_record(task: Task, now: float) -> Dict:
    scheduled = bool(task.start_at and task.start_at > now)
    return {
        "schema_version": SCHEMA_VERSION,
        "task_id": task.task_id,
        "name": task.name,
        "state": "SCHEDULED" if scheduled else "QUEUED",
        "message": "",
        "priority": task.priority,
        "map": task.map,
        "created_at": now,
        "started_at": None,
        "finished_at": None,
        "scheduled_for": task.start_at,
        "deadline_at": task.deadline_at,
        "run_index": 1,
        "run_count": task.repeat_count,
        "repeat_interval_sec": task.repeat_interval_sec,
        "callback_url": task.callback_url,
        "progress": {
            "steps_total": len(task.steps),
            "steps_done": 0,
            "steps_failed": 0,
            "steps_skipped": 0,
            "current_step": 0,
        },
        "steps": [_new_step_record(step) for step in task.steps],
    }


class InspectionManager:
    def __init__(self, hub=None, config: Optional[InspectionConfig] = None,
                 store: Optional[InspectionStore] = None, robot=None, reporter=None,
                 clock=time.time, sleep=time.sleep, log=None):
        self.hub = hub
        self.config = config or InspectionConfig(enabled=False)
        self._log = log or (lambda message: None)
        self.store = store or InspectionStore(self.config.storage_path())
        self._points_error = ""
        self.points = self._load_points(self.config.points_path())
        self.robot = robot or (RobotLink(hub) if hub is not None else RobotLinkStub())
        self.reporter = reporter or TaskReporter(
            self.store, enabled=self.config.callbacks_enabled, token=self.config.token,
            callback_hosts=self.config.callback_hosts, log=self._log)
        self._clock = clock
        self._sleep = sleep
        self._lock = threading.RLock()
        self._records: Dict[str, Dict] = {}
        self._tasks: Dict[str, Task] = {}
        self._cancel_events: Dict[str, threading.Event] = {}
        self._active_id = ""
        self._worker: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._mission_events: List[Dict] = []
        self._event_seq = 0
        self._last_mission_status: Dict = {}
        self._load_history()
        if self.config.enabled:
            self.start()

    def _load_points(self, path: str) -> PointLibrary:
        """A broken point library must not stop the console: report it in status() instead."""
        try:
            self._points_error = ""
            return PointLibrary.load(path)
        except (ValueError, OSError) as exc:
            self._points_error = str(exc)
            self._log("inspection point library unavailable: {}".format(exc))
            return PointLibrary(path=path or "")

    # -------------------------------------------------------------- lifecycle
    def start(self) -> None:
        if not self.config.enabled:
            return
        self.reporter.start()
        if self._worker is not None and self._worker.is_alive():
            return
        self._stop.clear()
        self._worker = threading.Thread(target=self._loop, name="inspection-worker", daemon=True)
        self._worker.start()

    def stop(self) -> None:
        self._stop_worker()
        self.reporter.stop()
        self.robot.shutdown()

    def _stop_worker(self) -> None:
        self._stop.set()
        if self._worker is not None:
            self._worker.join(timeout=5.0)
        self._worker = None

    # ------------------------------------------------------------ mission feed
    def on_mission_status(self, status: Dict) -> None:
        with self._lock:
            self._last_mission_status = dict(status or {})

    def on_mission_event(self, event: Dict) -> None:
        with self._lock:
            self._event_seq += 1
            self._mission_events.append(dict(event or {}, seq=self._event_seq))
            if len(self._mission_events) > 200:
                del self._mission_events[:-200]

    def _event_marker(self) -> int:
        with self._lock:
            return self._event_seq

    def _saw_event(self, marker: int, name: str) -> bool:
        with self._lock:
            return any(entry.get("event") == name and entry.get("seq", 0) > marker
                       for entry in self._mission_events)

    # -------------------------------------------------------------- submission
    def submit(self, payload) -> Dict:
        if not self.config.enabled:
            raise ValueError("inspection module is disabled (start the console without --no-inspection)")
        items = payload
        if isinstance(payload, dict):
            if "tasks" in payload:
                items = payload["tasks"]
            elif "task" in payload:
                items = [payload["task"]]
            else:
                items = [payload]
        if not isinstance(items, list) or not items:
            raise ValueError("expected a task object, a list of tasks, or {'tasks': [...]}")
        if len(items) > MAX_SUBMIT_BATCH:
            raise ValueError("at most {} tasks per request".format(MAX_SUBMIT_BATCH))

        accepted, rejected = [], []
        for index, raw in enumerate(items):
            now = self._clock()
            try:
                task = parse_task(raw, self.points, now)
            except TaskSchemaError as exc:
                rejected.append({"index": index, "error": str(exc)})
                continue
            with self._lock:
                existing = self._records.get(task.task_id)
                if existing is not None:
                    accepted.append({"task_id": task.task_id, "state": existing["state"],
                                     "duplicate": True})
                    continue
                pending = [record for record in self._records.values()
                           if record["state"] not in TERMINAL_TASK_STATES]
                if len(pending) >= self.config.max_queue:
                    rejected.append({"index": index,
                                     "error": "inspection queue is full ({} pending)".format(len(pending))})
                    continue
                record = _new_record(task, now)
                self._records[task.task_id] = record
                self._tasks[task.task_id] = task
            self.store.save_task({"schema_version": SCHEMA_VERSION, "task_id": task.task_id,
                                  "raw": raw, "normalized": task.to_dict()})
            self._persist(record)
            self._report(record, "ACCEPTED", "task accepted")
            accepted.append({"task_id": task.task_id, "state": record["state"],
                             "duplicate": False})
        return {"ok": bool(accepted), "accepted": accepted, "rejected": rejected}

    def cancel(self, task_id: str) -> Dict:
        with self._lock:
            record = self._records.get(task_id)
            if record is None:
                raise KeyError(task_id)
            if record["state"] in TERMINAL_TASK_STATES:
                return {"ok": False, "state": record["state"],
                        "message": "task already finished"}
            if record["state"] == "RUNNING":
                event = self._cancel_events.get(task_id)
                if event is not None:
                    event.set()
                self.robot.cancel_mission()
                return {"ok": True, "state": "CANCELLING",
                        "message": "cancellation requested"}
            record["state"] = "CANCELLED"
            record["finished_at"] = self._clock()
            record["message"] = "cancelled before start"
            self._tasks.pop(task_id, None)
        self._persist(record)
        self._report(record, "CANCELLED", record["message"])
        return {"ok": True, "state": "CANCELLED", "message": record["message"]}

    # ----------------------------------------------------------------- queries
    def status(self) -> Dict:
        with self._lock:
            active = self._records.get(self._active_id) if self._active_id else None
            queue = sorted((record for record in self._records.values()
                            if record["state"] in ("QUEUED", "SCHEDULED")),
                           key=lambda record: (-record["priority"], record["created_at"]))
            counts = {"queued": 0, "running": 0, "finished": 0}
            for record in self._records.values():
                if record["state"] in ("QUEUED", "SCHEDULED"):
                    counts["queued"] += 1
                elif record["state"] == "RUNNING":
                    counts["running"] += 1
                else:
                    counts["finished"] += 1
            snapshot = {
                "enabled": self.config.enabled,
                "active": self._summary(active) if active else None,
                "queue": [self._summary(record) for record in queue[:10]],
                "counts": counts,
                "points": dict(self.points.status(), error=self._points_error),
                "storage": {"dir": self.store.root, "free_bytes": self.store.free_bytes()},
                "reporter": self.reporter.status(),
                "actions": describe_actions(),
                "last_mission_status": copy.deepcopy(self._last_mission_status),
            }
        return snapshot

    def get_task(self, task_id: str) -> Dict:
        with self._lock:
            record = self._records.get(task_id)
            if record is None:
                raise KeyError(task_id)
            task = self._tasks.get(task_id)
            payload = copy.deepcopy(record)
            payload["task"] = task.to_dict() if task is not None else None
        payload["artifacts"] = self._artifacts(task_id)
        payload["last_report"] = (self.store.read_report(task_id, limit=1) or [None])[-1]
        return payload

    def list_tasks(self, state: Optional[str] = None, limit: int = 50) -> List[Dict]:
        wanted = None if state in (None, "", "all") else str(state).upper()
        with self._lock:
            records = [copy.deepcopy(record) for record in self._records.values() if
                       wanted is None or record["state"] == wanted]
        records.sort(key=lambda record: record["created_at"], reverse=True)
        return records[:max(1, min(int(limit), 200))]

    def report(self, task_id: str) -> List[Dict]:
        if task_id not in self._records:
            raise KeyError(task_id)
        return self.store.read_report(task_id)

    def points_list(self) -> List[Dict]:
        return self.points.list()

    def schema(self) -> Dict:
        return describe_schema()

    def artifact_path(self, task_id: str, name: str) -> str:
        if task_id not in self._records:
            raise KeyError(task_id)
        path = self.store.artifact_path(task_id, name)
        if not os.path.isfile(path):
            raise KeyError(name)
        return path

    def _artifacts(self, task_id: str) -> List[Dict]:
        items = []
        for artifact in self.store.list_artifacts(task_id):
            items.append({
                "name": artifact["name"],
                "size_bytes": artifact["size_bytes"],
                "created_at": artifact["created_at"],
                "url": "/api/inspection/tasks/{}/artifacts/{}".format(task_id, artifact["name"]),
            })
        return items

    # ------------------------------------------------------------------ auth
    def require_token(self, headers, query_token: Optional[str] = None) -> None:
        if not self.config.token:
            return
        provided = ""
        if headers is not None:
            provided = headers.get("X-Inspection-Token") or ""
        provided = provided or (query_token or "")
        if not provided or not hmac.compare_digest(str(provided), self.config.token):
            raise PermissionError("invalid or missing X-Inspection-Token")

    def operator_mission_blocked(self) -> Tuple[bool, str]:
        with self._lock:
            if self._active_id:
                return True, ("inspection task {} is running; cancel it before dispatching "
                              "operator waypoints".format(self._active_id))
        return False, ""

    # ----------------------------------------------------------------- worker
    def _loop(self) -> None:
        while not self._stop.wait(TICK_SEC):
            try:
                self._tick()
            except Exception as exc:  # keep the worker alive no matter what
                self._log("inspection worker error: {}".format(exc))
                self._sleep(0.5)

    def _tick(self) -> None:
        now = self._clock()
        due = None
        with self._lock:
            expired = [record for record in self._records.values()
                       if record["state"] in ("QUEUED", "SCHEDULED")
                       and record.get("deadline_at") and now > record["deadline_at"]]
            for record in expired:
                record["state"] = "FAILED"
                record["finished_at"] = now
                record["message"] = "deadline expired before the task started"
                self._tasks.pop(record["task_id"], None)
                self._report(record, "FAILED", record["message"])
            if self._active_id:
                return
            for record in self._records.values():
                if record["state"] == "SCHEDULED" and record["task_id"] in self._tasks:
                    if (record.get("scheduled_for") or 0.0) <= now:
                        record["state"] = "QUEUED"
                        record["message"] = ""
                        self._persist(record)
            candidates = [record for record in self._records.values()
                          if record["state"] == "QUEUED" and record["task_id"] in self._tasks]
            if candidates:
                candidates.sort(key=lambda record: (-record["priority"], record["created_at"]))
                due = self._tasks[candidates[0]["task_id"]]
        if due is not None:
            self._execute(due)

    def _execute(self, task: Task) -> None:
        with self._lock:
            record = self._records[task.task_id]
            cancel = threading.Event()
            self._cancel_events[task.task_id] = cancel
            self._active_id = task.task_id
            record["state"] = "RUNNING"
            record["started_at"] = self._clock()
            record["finished_at"] = None
            record["message"] = ""
            self._reset_steps(record)
        self._persist(record)
        self._report(record, "STARTED", "task started")

        outcome, reason = "SUCCEEDED", "all steps completed"
        try:
            ready, why = self.robot.motion_ready()
            if not ready:
                outcome, reason = "FAILED", why
            else:
                outcome, reason = self._run_steps(task, record, cancel)
        except Cancelled:
            outcome, reason = "CANCELLED", "cancelled by request"
        except Exception as exc:
            outcome, reason = "FAILED", "internal error: {}".format(exc)
            self._log("inspection task {} failed: {}".format(task.task_id, exc))
        finally:
            with self._lock:
                self._active_id = ""
                self._cancel_events.pop(task.task_id, None)
        self._finish(record, outcome, reason)

    def _run_steps(self, task: Task, record: Dict, cancel: threading.Event) -> Tuple[str, str]:
        any_done = False
        for index, step in enumerate(task.steps):
            if cancel.is_set():
                raise Cancelled()
            with self._lock:
                step_record = record["steps"][index]
                record["progress"]["current_step"] = index + 1
            outcome, reason = self._run_step(task, step, step_record, cancel)
            if outcome == "cancelled":
                raise Cancelled()
            with self._lock:
                if outcome == "done":
                    any_done = True
                    record["progress"]["steps_done"] += 1
                elif step_record["state"] == "SKIPPED":
                    record["progress"]["steps_failed"] += 1
                    record["progress"]["steps_skipped"] += 1
                else:
                    record["progress"]["steps_failed"] += 1
            self._persist(record)
            self._report(record, "STEP", "step '{}': {}".format(step.step_id, reason))
            if outcome != "done" and task.on_step_failure == "abort":
                return (("PARTIAL" if any_done else "FAILED"),
                        "aborted after step '{}': {}".format(step.step_id, reason))
        if any_done and not all(step["state"] == "DONE" for step in record["steps"]):
            return "PARTIAL", "one or more steps did not complete"
        if not any_done:
            return "FAILED", "no step completed"
        return "SUCCEEDED", "all steps completed"

    def _run_step(self, task: Task, step, step_record: Dict,
                  cancel: threading.Event) -> Tuple[str, str]:
        with self._lock:
            step_record["state"] = "NAVIGATING"
            step_record["message"] = ""
        self._persist(self._records[task.task_id])

        outcome, reason = self._navigate(task, step, step_record, cancel)
        if outcome == "cancelled":
            with self._lock:
                step_record["state"] = "CANCELLED"
                step_record["message"] = reason
            return "cancelled", reason
        if outcome != "arrived":
            with self._lock:
                step_record["state"] = "SKIPPED" if outcome == "skipped" else "FAILED"
                step_record["message"] = reason
            return "failed", reason

        with self._lock:
            step_record["state"] = "ACTING"
            step_record["arrived_at"] = self._clock()
            step_record["pose"] = self.robot.pose()
        self._persist(self._records[task.task_id])

        ctx = ActionContext(self.robot, task, step, self.store, cancel)
        failures = []
        try:
            for sensor in step.sensors:
                entry = run_sensor(ctx, sensor)
                with self._lock:
                    step_record["sensors"].append(entry)
                if entry["state"] != "OK":
                    failures.append("sensor {}: {}".format(sensor, entry["message"]))
                self._persist(self._records[task.task_id])
            for action in step.actions:
                entry = run_action(ctx, action)
                with self._lock:
                    step_record["actions"].append(entry)
                if entry["state"] != "OK":
                    failures.append("action {}: {}".format(entry["name"], entry["message"]))
                self._persist(self._records[task.task_id])
        except Cancelled:
            with self._lock:
                step_record["state"] = "CANCELLED"
                step_record["message"] = "cancelled during step actions"
            raise

        with self._lock:
            if failures:
                step_record["state"] = "PARTIAL"
                step_record["message"] = "; ".join(failures)[:MAX_MESSAGE_CHARS]
            else:
                step_record["state"] = "DONE"
                step_record["message"] = "arrived and completed all sensors/actions"
            final_state = step_record["state"]
            final_message = step_record["message"]
        return ("done" if final_state == "DONE" else "failed"), final_message

    def _navigate(self, task: Task, step, step_record: Dict,
                  cancel: threading.Event) -> Tuple[str, str]:
        attempts = step.retries + 1
        last_reason = "navigation failed"
        for attempt in range(1, attempts + 1):
            if cancel.is_set():
                return "cancelled", "cancelled by request"
            with self._lock:
                step_record["attempts"] = attempt
            ok, message, mission_id = self.robot.start_mission([step.waypoint()])
            if not ok:
                last_reason = "mission dispatch rejected: {}".format(message)
                if attempt < attempts:
                    self._sleep(RETRY_BACKOFF_SEC)
                continue
            marker = self._event_marker()
            outcome, reason = self._wait_mission(mission_id, marker, step.timeout_sec, cancel)
            if outcome in ("arrived", "cancelled", "estop", "stalled"):
                return outcome, reason
            last_reason = reason or outcome
            self.robot.cancel_mission()
            if attempt < attempts:
                self._sleep(RETRY_BACKOFF_SEC)
        return "failed", last_reason

    def _wait_mission(self, mission_id: str, marker: int, timeout_sec: float,
                      cancel: threading.Event) -> Tuple[str, str]:
        started = time.monotonic()
        deadline = started + max(1.0, float(timeout_sec))
        while True:
            if cancel.is_set():
                self.robot.cancel_mission()
                return "cancelled", "cancelled by request"
            if self.robot.estop_locked():
                self.robot.cancel_mission()
                return "estop", "emergency stop latched during navigation"
            status = self.robot.mission_status()
            state = status.get("state") if status.get("mission_id") == mission_id else None
            if state == "COMPLETED":
                return "arrived", "arrived"
            if state in ("FAILED", "CANCELLED"):
                return state.lower(), "mission_manager reported {}".format(state)
            if self._saw_event(marker, "WAYPOINT_SKIPPED"):
                return "skipped", "waypoint skipped by mission_manager"
            if (time.monotonic() - started) > STALL_GRACE_SEC and self.robot.stall_alarm():
                self.robot.cancel_mission()
                return "stalled", "stall alarm triggered during navigation"
            if time.monotonic() >= deadline:
                self.robot.cancel_mission()
                return "timeout", "navigation timed out after {:g}s".format(float(timeout_sec))
            self._sleep(MISSION_POLL_SEC)

    def _finish(self, record: Dict, state: str, reason: str) -> None:
        now = self._clock()
        with self._lock:
            record["message"] = (reason or "")[:MAX_MESSAGE_CHARS]
            if state in ("SUCCEEDED", "PARTIAL") and \
                    record["run_index"] < record.get("run_count", 1):
                record["run_index"] += 1
                record["scheduled_for"] = now + float(record.get("repeat_interval_sec") or 0.0)
                record["state"] = "SCHEDULED"
                record["finished_at"] = now
                self._reset_steps(record)
                self._persist(record)
                self._report(record, "RESCHEDULED",
                             "run {} of {} finished; next run scheduled".format(
                                 record["run_index"] - 1, record["run_count"]))
                return
            record["state"] = state if state in TASK_STATES else "FAILED"
            record["finished_at"] = now
            self._tasks.pop(record["task_id"], None)
        self._persist(record)
        self._report(record, "FINISHED", record["message"])
        try:
            self.store.prune(self.config.max_tasks_kept)
        except Exception as exc:
            self._log("inspection prune failed: {}".format(exc))
        with self._lock:
            self._prune_memory()

    def _prune_memory(self) -> None:
        terminal = [(record.get("finished_at") or 0.0, task_id)
                    for task_id, record in self._records.items()
                    if record["state"] in TERMINAL_TASK_STATES]
        overflow = len(terminal) - self.config.max_tasks_kept
        if overflow <= 0:
            return
        terminal.sort()
        for _, task_id in terminal[:overflow]:
            self._records.pop(task_id, None)

    @staticmethod
    def _reset_steps(record: Dict) -> None:
        record["progress"] = {
            "steps_total": len(record["steps"]),
            "steps_done": 0,
            "steps_failed": 0,
            "steps_skipped": 0,
            "current_step": 0,
        }
        for step_record in record["steps"]:
            step_record.update({
                "state": "PENDING", "message": "", "attempts": 0,
                "arrived_at": None, "pose": None, "sensors": [], "actions": [],
            })

    def _persist(self, record: Dict) -> None:
        with self._lock:
            snapshot = copy.deepcopy(record)
        self.store.save_state(snapshot)

    def _report(self, record: Dict, event: str, message: str) -> None:
        with self._lock:
            snapshot = copy.deepcopy(record)
        payload = self._payload(snapshot, event, message)
        try:
            self.store.append_report(snapshot["task_id"], payload)
        except OSError as exc:
            self._log("cannot append inspection report: {}".format(exc))
        terminal = snapshot["state"] in TERMINAL_TASK_STATES
        if terminal or self.config.report_progress:
            url = snapshot.get("callback_url") or self.config.default_callback_url
            self.reporter.submit(url, payload)

    def _payload(self, record: Dict, event: str, message: str) -> Dict:
        task_id = record["task_id"]
        return {
            "schema_version": SCHEMA_VERSION,
            "event": event,
            "task_id": task_id,
            "name": record["name"],
            "state": record["state"],
            "message": message,
            "priority": record["priority"],
            "map": record["map"],
            "run_index": record["run_index"],
            "run_count": record.get("run_count", 1),
            "created_at": _iso(record.get("created_at")),
            "started_at": _iso(record.get("started_at")),
            "finished_at": _iso(record.get("finished_at")),
            "scheduled_for": _iso(record.get("scheduled_for")),
            "deadline_at": _iso(record.get("deadline_at")),
            "progress": copy.deepcopy(record["progress"]),
            "steps": copy.deepcopy(record["steps"]),
            "artifacts": self._artifacts(task_id),
            "robot": {"pose": self.robot.pose()},
            "reported_at": self._clock(),
        }

    @staticmethod
    def _summary(record: Dict) -> Dict:
        current = record["progress"].get("current_step") or 0
        return {
            "task_id": record["task_id"],
            "name": record["name"],
            "state": record["state"],
            "message": record["message"],
            "priority": record["priority"],
            "created_at": record.get("created_at"),
            "scheduled_for": record.get("scheduled_for"),
            "run_index": record["run_index"],
            "run_count": record.get("run_count", 1),
            "progress": copy.deepcopy(record["progress"]),
            "current_step_id": (record["steps"][current - 1]["step_id"]
                                if 0 < current <= len(record["steps"]) else ""),
        }

    # ---------------------------------------------------------------- restart
    def _load_history(self) -> None:
        now = self._clock()
        for state in self.store.load_states():
            task_id = state["task_id"]
            if state.get("state") == "RUNNING":
                state["state"] = "INTERRUPTED"
                state["message"] = "console restarted while this task was running"
                state["finished_at"] = now
                self.store.save_state(state)
                self.store.append_report(task_id, self._payload(state, "INTERRUPTED",
                                                                state["message"]))
            self._records[task_id] = state
            if state.get("state") not in ("QUEUED", "SCHEDULED"):
                continue
            stored = self.store.load_task(task_id) or {}
            try:
                self._tasks[task_id] = parse_task(stored.get("raw"), self.points, now)
            except (TaskSchemaError, KeyError) as exc:
                state["state"] = "FAILED"
                state["finished_at"] = now
                state["message"] = "stored task can no longer be validated: {}".format(exc)
                self.store.save_state(state)

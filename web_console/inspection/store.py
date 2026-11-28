"""File-per-task persistence for inspection tasks, artifacts and reports.

Everything lives under the configured storage directory:

    <root>/tasks/<task_id>/task.json         normalized task (source of truth)
    <root>/tasks/<task_id>/state.json        latest runtime snapshot
    <root>/tasks/<task_id>/report.jsonl      append-only report history
    <root>/tasks/<task_id>/artifacts/...      captured images and metrics
    <root>/pending_reports.jsonl             reports the backend did not accept
"""

import json
import os
import re
import shutil
import tempfile
import time
from typing import Dict, List, Optional

_SAFE_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")


def _safe_name(name: str) -> str:
    candidate = str(name or "").strip()
    if not candidate or candidate in (".", "..") or not _SAFE_NAME.match(candidate):
        raise ValueError("unsafe file name '{}'".format(name))
    return candidate


class InspectionStore:
    def __init__(self, root: str):
        self.root = os.path.abspath(os.path.expanduser(os.path.expandvars(root)))
        self.tasks_dir = os.path.join(self.root, "tasks")
        self.pending_path = os.path.join(self.root, "pending_reports.jsonl")
        os.makedirs(self.tasks_dir, exist_ok=True)

    # ---------------------------------------------------------------- paths
    def task_dir(self, task_id: str) -> str:
        return os.path.join(self.tasks_dir, _safe_name(task_id))

    def artifact_dir(self, task_id: str, create: bool = True) -> str:
        path = os.path.join(self.task_dir(task_id), "artifacts")
        if create:
            os.makedirs(path, exist_ok=True)
        return path

    def artifact_path(self, task_id: str, name: str) -> str:
        return os.path.join(self.artifact_dir(task_id), _safe_name(name))

    # ---------------------------------------------------------------- json io
    @staticmethod
    def _write_json(path: str, payload: Dict) -> None:
        directory = os.path.dirname(path)
        os.makedirs(directory, exist_ok=True)
        handle, temp_path = tempfile.mkstemp(dir=directory, prefix=".tmp-", suffix=".json")
        try:
            with os.fdopen(handle, "w", encoding="utf-8") as stream:
                json.dump(payload, stream, ensure_ascii=False, indent=2, sort_keys=True)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temp_path, path)
        except Exception:
            try:
                os.unlink(temp_path)
            except OSError:
                pass
            raise

    @staticmethod
    def _read_json(path: str, default=None):
        try:
            with open(path, "r", encoding="utf-8") as stream:
                return json.load(stream)
        except (OSError, ValueError):
            return default

    # ---------------------------------------------------------------- tasks
    def save_task(self, task: Dict) -> None:
        self._write_json(os.path.join(self.task_dir(task["task_id"]), "task.json"), task)

    def load_task(self, task_id: str) -> Optional[Dict]:
        return self._read_json(os.path.join(self.task_dir(task_id), "task.json"))

    def save_state(self, state: Dict) -> None:
        self._write_json(os.path.join(self.task_dir(state["task_id"]), "state.json"), state)

    def load_state(self, task_id: str) -> Optional[Dict]:
        return self._read_json(os.path.join(self.task_dir(task_id), "state.json"))

    def load_states(self) -> List[Dict]:
        states = []
        try:
            entries = sorted(os.listdir(self.tasks_dir))
        except OSError:
            return states
        for entry in entries:
            state = self._read_json(os.path.join(self.tasks_dir, entry, "state.json"))
            if isinstance(state, dict) and state.get("task_id"):
                states.append(state)
        return states

    def delete_task(self, task_id: str) -> None:
        shutil.rmtree(self.task_dir(task_id), ignore_errors=True)

    # ---------------------------------------------------------------- artifacts
    def save_artifact(self, task_id: str, name: str, data: bytes) -> Dict:
        path = self.artifact_path(task_id, name)
        with open(path, "wb") as stream:
            stream.write(data)
        return {"name": os.path.basename(path), "path": path, "size_bytes": len(data),
                "created_at": time.time()}

    def list_artifacts(self, task_id: str) -> List[Dict]:
        directory = self.artifact_dir(task_id, create=False)
        items = []
        try:
            entries = sorted(os.listdir(directory))
        except OSError:
            return items
        for entry in entries:
            path = os.path.join(directory, entry)
            if not os.path.isfile(path):
                continue
            try:
                stat = os.stat(path)
            except OSError:
                continue
            items.append({"name": entry, "path": path, "size_bytes": stat.st_size,
                          "created_at": stat.st_mtime})
        return items

    # ---------------------------------------------------------------- reports
    def append_report(self, task_id: str, entry: Dict) -> None:
        path = os.path.join(self.task_dir(task_id), "report.jsonl")
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "a", encoding="utf-8") as stream:
            stream.write(json.dumps(entry, ensure_ascii=False, sort_keys=True) + "\n")
            stream.flush()

    def read_report(self, task_id: str, limit: int = 200) -> List[Dict]:
        path = os.path.join(self.task_dir(task_id), "report.jsonl")
        entries: List[Dict] = []
        try:
            with open(path, "r", encoding="utf-8") as stream:
                for line in stream:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        entries.append(json.loads(line))
                    except ValueError:
                        continue
        except OSError:
            return []
        return entries[-max(1, int(limit)):]

    def spool_pending(self, payload: Dict) -> None:
        os.makedirs(os.path.dirname(self.pending_path), exist_ok=True)
        with open(self.pending_path, "a", encoding="utf-8") as stream:
            stream.write(json.dumps(payload, ensure_ascii=False, sort_keys=True) + "\n")
            stream.flush()

    def drain_pending(self, limit: int = 20) -> List[Dict]:
        """Removes and returns spooled reports so they can be retried."""
        if not os.path.isfile(self.pending_path):
            return []
        pending: List[Dict] = []
        remaining: List[str] = []
        with open(self.pending_path, "r", encoding="utf-8") as stream:
            lines = stream.readlines()
        for line in lines:
            if len(pending) >= limit:
                remaining.append(line)
                continue
            line = line.strip()
            if not line:
                continue
            try:
                pending.append(json.loads(line))
            except ValueError:
                continue
        if remaining:
            with open(self.pending_path, "w", encoding="utf-8") as stream:
                stream.writelines(remaining)
        else:
            try:
                os.unlink(self.pending_path)
            except OSError:
                pass
        return pending

    def prune(self, keep: int = 200) -> None:
        """Drops the oldest finished tasks so an unattended robot cannot fill the disk."""
        states = []
        for state in self.load_states():
            if state.get("state") in ("QUEUED", "SCHEDULED", "RUNNING"):
                continue
            states.append((float(state.get("created_at") or 0.0), state["task_id"]))
        states.sort(reverse=True)
        for _, task_id in states[max(0, keep):]:
            self.delete_task(task_id)

    def free_bytes(self) -> int:
        try:
            usage = shutil.disk_usage(self.root)
            return int(usage.free)
        except OSError:
            return 0

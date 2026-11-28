"""HTTP surface for the inspection module (mounted under /api/inspection).

Kept out of `web_console.server` so the console's request handler stays a thin
router: handlers here return either a JSON payload or a raw (bytes, mimetype)
artifact response.
"""

import json
from urllib.parse import unquote

MAX_LIST_LIMIT = 200


class Rejected(ValueError):
    """Validation failure that carries the full result so the backend sees every index."""

    def __init__(self, message: str, payload: dict):
        super().__init__(message)
        self.payload = payload


def _first(query, key, default=""):
    values = (query or {}).get(key) or []
    return str(values[0]) if values else default


def _artifact_mimetype(name: str) -> str:
    lowered = name.lower()
    if lowered.endswith((".jpg", ".jpeg")):
        return "image/jpeg"
    if lowered.endswith(".json"):
        return "application/json; charset=utf-8"
    if lowered.endswith(".png"):
        return "image/png"
    return "application/octet-stream"


def _task_tail(path: str):
    prefix = "/api/inspection/tasks"
    remainder = path[len(prefix):].strip("/")
    if not remainder:
        return None, []
    parts = remainder.split("/")
    return unquote(parts[0]), [unquote(part) for part in parts[1:]]


def handle_get(manager, path: str, query, headers):
    path = path.rstrip("/") or "/api/inspection"
    manager.require_token(headers, _first(query, "token") or None)

    if path in ("/api/inspection", "/api/inspection/status"):
        return manager.status()
    if path == "/api/inspection/schema":
        return manager.schema()
    if path == "/api/inspection/points":
        return {"ok": True, "points": manager.points_list()}
    if path == "/api/inspection/tasks":
        limit = _first(query, "limit", "50")
        try:
            limit = max(1, min(int(limit), MAX_LIST_LIMIT))
        except ValueError:
            raise ValueError("limit must be an integer")
        state = _first(query, "state") or None
        return {"ok": True, "tasks": manager.list_tasks(state=state, limit=limit)}

    if not path.startswith("/api/inspection/tasks/"):
        raise KeyError(path)
    task_id, tail = _task_tail(path)
    if task_id is None:
        raise KeyError(path)
    if not tail:
        return manager.get_task(task_id)
    if tail == ["report"]:
        return {"ok": True, "task_id": task_id, "entries": manager.report(task_id)}
    if tail == ["artifacts"]:
        return {"ok": True, "task_id": task_id, "artifacts": manager.get_task(task_id)["artifacts"]}
    if len(tail) == 2 and tail[0] == "artifacts":
        path_on_disk = manager.artifact_path(task_id, tail[1])
        with open(path_on_disk, "rb") as handle:
            data = handle.read()
        return data, _artifact_mimetype(tail[1]), ()
    raise KeyError(path)


def handle_post(manager, path: str, body: bytes, headers, query):
    path = path.rstrip("/") or "/api/inspection"
    manager.require_token(headers, _first(query, "token") or None)
    payload = json.loads(body.decode("utf-8")) if body else {}

    if path == "/api/inspection/tasks":
        result = manager.submit(payload)
        if not result.get("accepted"):
            raise Rejected("no task was accepted", result)
        return result
    if not path.startswith("/api/inspection/tasks/"):
        raise KeyError(path)
    task_id, tail = _task_tail(path)
    if task_id is None:
        raise KeyError(path)
    if tail == ["cancel"]:
        return manager.cancel(task_id)
    raise KeyError(path)

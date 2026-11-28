"""Delivers inspection reports to the backend off the executor thread.

Reports are always written to the task's `report.jsonl`; the HTTP push is best
effort with a local spool so a backend outage never blocks the robot.
"""

import json
import queue
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from typing import Dict, Optional

USER_AGENT = "ons2-inspection/1.0"
MAX_SPOOL_ATTEMPTS = 5


class TaskReporter:
    def __init__(self, store, enabled: bool = True, timeout: float = 4.0, token: str = "",
                 callback_hosts=(), log=None):
        self.store = store
        self.enabled = bool(enabled)
        self.timeout = float(timeout)
        self.token = token or ""
        self.callback_hosts = tuple(host.lower() for host in (callback_hosts or ()))
        self._log = log or (lambda message: None)
        self._queue: "queue.Queue" = queue.Queue(maxsize=200)
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._last_lock = threading.Lock()
        self._last_result: Dict = {}
        self._delivered = 0
        self._failed = 0

    # -------------------------------------------------------------- lifecycle
    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name="inspection-reporter", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        try:
            self._queue.put_nowait(None)
        except queue.Full:
            pass
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._thread = None

    # --------------------------------------------------------------- submission
    def submit(self, url: str, payload: Dict) -> None:
        if not url or not self.enabled:
            return
        try:
            self._queue.put_nowait((url, payload))
        except queue.Full:
            self.store.spool_pending({"url": url, "payload": payload, "attempts": 0})

    def wait_idle(self, timeout: float = 3.0) -> bool:
        """Blocks until the queue is drained (used by tests and shutdown)."""
        deadline = time.monotonic() + max(0.0, timeout)
        while time.monotonic() < deadline:
            if self._queue.empty():
                return True
            time.sleep(0.02)
        return self._queue.empty()

    # ------------------------------------------------------------------ worker
    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                item = self._queue.get(timeout=0.2)
            except queue.Empty:
                self._flush_pending(limit=2)
                continue
            if item is None:
                break
            url, payload = item
            self._deliver(url, payload, attempts=0)

    def _flush_pending(self, limit: int) -> None:
        for entry in self.store.drain_pending(limit=limit):
            attempts = int(entry.get("attempts") or 0) + 1
            url = str(entry.get("url") or "")
            payload = entry.get("payload") or {}
            if attempts > MAX_SPOOL_ATTEMPTS or not url:
                self._log("inspection report for {} dropped after {} attempts".format(
                    payload.get("task_id", "?"), attempts))
                continue
            result = self._deliver(url, payload, attempts=attempts)
            if not result.get("delivered"):
                entry["attempts"] = attempts
                self.store.spool_pending(entry)

    def _allowed(self, url: str):
        parsed = urllib.parse.urlparse(url)
        if parsed.scheme not in ("http", "https"):
            return False, "callback scheme must be http or https"
        if self.callback_hosts:
            host = (parsed.hostname or "").lower()
            if host not in self.callback_hosts:
                return False, "callback host '{}' is not in the allowlist".format(host)
        return True, ""

    def _deliver(self, url: str, payload: Dict, attempts: int = 0) -> Dict:
        allowed, reason = self._allowed(url)
        if not allowed:
            result = {"delivered": False, "url": url, "error": reason, "at": time.time()}
            self._remember(result)
            return result
        headers = {"Content-Type": "application/json; charset=utf-8", "User-Agent": USER_AGENT}
        if self.token:
            headers["X-Inspection-Token"] = self.token
        request = urllib.request.Request(
            url, data=json.dumps(payload, ensure_ascii=False).encode("utf-8"),
            headers=headers, method="POST")
        try:
            with urllib.request.urlopen(request, timeout=self.timeout) as response:
                response.read(512)
                status = int(getattr(response, "status", 0) or 0)
            delivered = 200 <= status < 300
            result = {"delivered": delivered, "url": url, "status": status, "at": time.time(),
                      "attempts": attempts}
            if not delivered:
                result["error"] = "HTTP {}".format(status)
        except (urllib.error.URLError, urllib.error.HTTPError, OSError, ValueError) as exc:
            result = {"delivered": False, "url": url, "error": str(exc), "at": time.time(),
                      "attempts": attempts}
        if result["delivered"]:
            self._delivered += 1
        else:
            self._failed += 1
        self._remember(result)
        return result

    def _remember(self, result: Dict) -> None:
        with self._last_lock:
            self._last_result = dict(result)
        if not result.get("delivered"):
            self._log("inspection report delivery failed: {}".format(result.get("error")))

    def status(self) -> Dict:
        with self._last_lock:
            last = dict(self._last_result)
        return {
            "enabled": self.enabled,
            "pending": self._queue.qsize(),
            "delivered": self._delivered,
            "failed": self._failed,
            "last": last,
            "token_configured": bool(self.token),
            "callback_hosts": list(self.callback_hosts),
        }

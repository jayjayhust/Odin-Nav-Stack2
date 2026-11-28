"""Robot facade used by the inspection executor and action handlers.

The console's ROS 1 / ROS 2 adapters bind raw topics/services to the hub; this
class exposes the small, typed surface the inspection module needs and keeps
adapter details (camera gating, mission services) out of the state machine.
"""

import threading
import time
from concurrent.futures import ThreadPoolExecutor, TimeoutError as FutureTimeout

CAPTURE_TIMEOUT_SEC = 3.0
SPEAK_TIMEOUT_SEC = 25.0


class RobotLink:
    def __init__(self, hub):
        self.hub = hub
        self._speak_pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="inspection-tts")

    # ------------------------------------------------------------------ state
    def pose(self):
        snapshot = self.hub.console.snapshot()
        return snapshot.get("pose")

    def mission_status(self):
        return (self.hub.mission.get_status() or {}).get("status") or {}

    def chassis(self):
        status = self.hub.mission.get_status() or {}
        return status.get("chassis_supported"), status.get("chassis_enabled")

    def estop_locked(self) -> bool:
        return bool((self.hub.nav.get_status() or {}).get("estop_locked"))

    def stall_alarm(self) -> bool:
        return bool((self.hub.stall.evaluate() or {}).get("alarm"))

    def motion_ready(self):
        """Returns (ok, reason). Unsupported chassis enable (no bridge) is not a blocker."""
        supported, enabled = self.chassis()
        if supported and enabled is False:
            return False, "chassis motion is disabled (enable it in the web console first)"
        if self.estop_locked():
            return False, "emergency stop is latched; release it before dispatching tasks"
        return True, ""

    # --------------------------------------------------------------- missions
    def start_mission(self, points):
        """Dispatches one single-waypoint mission; returns (ok, message, mission_id)."""
        result = self.hub.mission.start(list(points), loop=False, temporary=False, source="inspection")
        return (bool(result.get("ok")), str(result.get("message") or ""),
                str(result.get("mission_id") or ""))

    def cancel_mission(self):
        return self.hub.mission.command("cancel")

    # ----------------------------------------------------------------- sensors
    def sample_scan(self, samples: int = 3, interval_sec: float = 0.2):
        sampler = getattr(self.hub, "sample_scan", None)
        if sampler is None:
            return None
        return sampler(samples=int(samples), interval_sec=float(interval_sec))

    def capture_jpeg(self, timeout: float = CAPTURE_TIMEOUT_SEC):
        """Grabs one frame, temporarily enabling the on-demand camera if needed."""
        gate = self.hub.camera
        was_enabled = gate.enabled
        if not was_enabled:
            gate.set_enabled(True, fps=2.0)
        try:
            deadline = time.monotonic() + max(0.5, float(timeout))
            seq = gate.latest_seq
            while True:
                jpeg, _, stamp = gate.wait_for_next_frame(seq, timeout=0.5)
                if jpeg:
                    return jpeg, stamp
                if time.monotonic() >= deadline:
                    return None, 0.0
        finally:
            if not was_enabled:
                gate.set_enabled(False)

    def speak(self, text: str, volume: int = 100):
        try:
            future = self._speak_pool.submit(self.hub.intercom.speak_tts, text, volume)
            future.result(timeout=SPEAK_TIMEOUT_SEC)
            return True, "spoken"
        except FutureTimeout:
            return False, "TTS playback timed out after {:g}s".format(SPEAK_TIMEOUT_SEC)
        except Exception as exc:
            return False, str(exc) or exc.__class__.__name__

    def shutdown(self):
        self._speak_pool.shutdown(wait=False)


class RobotLinkStub:
    """Minimal robot used by tests and by a console with no ROS adapter bound.

    `mission_state` is what `mission_status()` reports for the mission it just
    accepted, so tests can drive arrivals and failures without a ROS graph.
    """

    def __init__(self, hub=None):
        self._lock = threading.Lock()
        self.pose_value = None
        self.started = []
        self.cancels = 0
        self.frames = []
        self.scans = []
        self.spoken = []
        self.mission_state = "COMPLETED"
        self.mission_id = ""

    def pose(self):
        return self.pose_value

    def mission_status(self):
        with self._lock:
            if not self.mission_id:
                return {}
            return {"mission_id": self.mission_id, "state": self.mission_state}

    def chassis(self):
        return False, None

    def estop_locked(self):
        return False

    def stall_alarm(self):
        return False

    def motion_ready(self):
        return True, ""

    def start_mission(self, points):
        with self._lock:
            self.started.append(list(points))
            self.mission_id = "mission_stub_{}".format(len(self.started))
            return True, "accepted", self.mission_id

    def cancel_mission(self):
        with self._lock:
            self.cancels += 1
        return True, "cancelled"

    def sample_scan(self, samples=3, interval_sec=0.2):
        if not self.scans:
            return None
        return self.scans.pop(0)

    def capture_jpeg(self, timeout=CAPTURE_TIMEOUT_SEC):
        if not self.frames:
            return None, 0.0
        return self.frames.pop(0), time.time()

    def speak(self, text, volume=100):
        self.spoken.append((text, volume))
        return True, "ok"

    def shutdown(self):
        pass

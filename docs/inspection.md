# Inspection tasks (structured tasks from an inspection backend)

**English** | [中文](inspection.zh-CN.md)

The inspection module lets an external **inspection backend** push structured patrol tasks to the
robot: *when to run, which points to visit, which sensors to fire, which actions to perform*. The
robot queues them, drives to each point with the normal navigation stack, parks, runs the sensors and
actions, stores the evidence (images, lidar metrics) locally, and reports progress and results back.

It is a client of the existing stack, not a new stack:

- Motion is delegated to `mission_manager` (`/waypoint_manager/start_mission`), one
  **single-waypoint mission per step**. The robot only ever moves between waypoints; every
  sensor/action runs while it is parked.
- The module lives inside `web_console/` (`web_console/inspection/`) so the same code runs on ROS 2
  (SW01) and ROS 1 (Go2, Dobot Lumo) through the existing adapters, and the backend needs nothing but
  the console's single HTTP port.
- No ROS messages, services or C++ changes were added.

---

## Execution model

```
backend ──POST /api/inspection/tasks──► queue (persisted)
                                          │
                              scheduler ──┴─ priority, then FIFO; honours start_at/deadline_at
                                          │
                                          ▼
                     for each step: start_mission([waypoint]) → wait COMPLETED
                                          │
                                    robot parked
                                          │
                              sensors → actions → artifacts + metrics
                                          │
                                          ▼
                       report.jsonl + POST callback_url (X-Inspection-Token)
```

Why one mission per step instead of one mission per task:

- **Retries and skipping** — a failed waypoint is retried and then marked `SKIPPED` without touching
  the rest of the task.
- **Floor safety** — the multi-floor transit logic of `mission_manager` applies per waypoint, exactly
  as for an operator mission.
- **No dwell/action race** — `dwell_sec` ends inside `mission_manager`; actions start only after the
  mission is `COMPLETED`.
- **Predictable cancellation** — cancelling stops the current mission and aborts the remaining steps.

The cost is one global replan per step, which is negligible next to the inspection actions.

---

## Enabling it

The module is **on by default**. It is disabled with `--no-inspection`, in the config file, or with
`ONS2_INSPECTION_ENABLE=0`.

```bash
# ROS 2 (SW01)
python3 -m web_console.run --port 18090 --inspection-token "$ONS2_INSPECTION_TOKEN"

# ROS 1 (Go2 / Dobot Lumo) — same flags
ROS_VERSION=1 python3 -m web_console.run --port 18090 --inspection-token "$ONS2_INSPECTION_TOKEN"
```

The systemd unit (`scripts/systemd/web-console.service`) starts the console without a token; add
`--inspection-token` there (or an `Environment=` line) before exposing the port beyond a trusted LAN.

| Flag | Default | Meaning |
|---|---|---|
| `--inspection` / `--no-inspection` | on | Accept tasks and run the executor thread. |
| `--inspection-dir` | `~/ons2_inspection` | Task, artifact and report storage root. |
| `--inspection-points` | `<repo>/inspection_points.yaml` | Point library (see below). |
| `--inspection-token` | empty (no auth) | Shared secret required on every `/api/inspection*` request. |
| `--inspection-callback` | empty | Default report callback URL for tasks that carry none. |
| `--inspection-no-callback` | callbacks on | Keep reports local; never POST to the backend. |
| `--inspection-callback-host` | empty (any host) | Allowlist a callback host; repeatable. |
| `--inspection-config` | none | YAML file with an `inspection:` section. |
| `--scan-topic` | `/scan_360` | `sensor_msgs/LaserScan` used by `lidar_scan` and `measure_scan`. |

Environment variables: `ONS2_INSPECTION_ENABLE`, `ONS2_INSPECTION_DIR`, `ONS2_INSPECTION_POINTS`,
`ONS2_INSPECTION_TOKEN`, `ONS2_INSPECTION_CALLBACK`, `ONS2_INSPECTION_CALLBACKS`,
`ONS2_INSPECTION_CONFIG`. Precedence is **CLI > environment > config file > defaults**.

```yaml
# inspection.yaml  (pass with --inspection-config)
inspection:
  enabled: true
  storage_dir: /var/lib/ons2/inspection
  points_file: /home/robot/inspection_points.yaml
  token: change-me
  callbacks_enabled: true
  default_callback_url: https://inspection.example.com/api/robot/report
  callback_hosts: [inspection.example.com]
  max_queue: 32
  max_tasks_kept: 200
  report_progress: false
```

> **Token advice.** Without `--inspection-token` any client on the console's port can enqueue tasks,
> i.e. drive the robot. Set a token on any robot reachable from an untrusted network. Send it as the
> `X-Inspection-Token` header or as `?token=` (the query form exists so a browser can open task
> detail URLs directly).

---

## Submitting a task

```bash
curl -sS -X POST http://192.168.1.20:18090/api/inspection/tasks \
  -H 'Content-Type: application/json' \
  -H "X-Inspection-Token: $ONS2_INSPECTION_TOKEN" \
  -d @docs/inspection/task.example.json
```

Accepted shapes: a single task object, a bare list of tasks, `{"tasks": [...]}` or `{"task": {...}}`
(at most 50 tasks per request).

```json
{
  "accepted": [{"task_id": "night-1f-20261123", "state": "QUEUED", "duplicate": false}],
  "rejected": [{"index": 1, "error": "steps[0].target needs a point_id or an x/y pair"}],
  "ok": true
}
```

- `200` with `rejected` non-empty means a **partial** accept: the listed tasks were not enqueued, the
  rest were.
- `400` with the same body shape means **nothing** was accepted (`ok: false`), so retrying the whole
  batch is safe.
- Submitting a `task_id` that is already known returns `duplicate: true` and changes nothing — retries
  after a network timeout are safe.

### Task schema

```json
{
  "schema_version": 1,
  "task_id": "night-1f-20261123",
  "name": "夜间巡检 · 一层",
  "priority": 10,
  "map": "1F",
  "start_at": "2026-11-23T22:00:00+08:00",
  "deadline_at": "2026-11-23T23:30:00+08:00",
  "repeat": {"count": 2, "interval_sec": 1800},
  "on_step_failure": "continue",
  "callback_url": "https://inspection.example.com/api/robot/report",
  "metadata": {"ticket": "INS-2026-1131", "operator": "ops-2"},
  "steps": [
    {
      "step_id": "lobby",
      "name": "大厅",
      "target": {"point_id": "lobby_door"},
      "timeout_sec": 180,
      "retries": 1,
      "sensors": ["camera", "lidar_scan"],
      "actions": [
        {"type": "speak", "params": {"text": "夜间巡检，请勿靠近", "volume": 80}},
        {"type": "capture_burst", "params": {"count": 3, "interval_sec": 0.5}}
      ]
    }
  ]
}
```

| Task field | Required | Notes |
|---|---|---|
| `task_id` | no | `[A-Za-z0-9][A-Za-z0-9._:-]{0,63}`. Generated (`insp_<ms>_<seq>`) when omitted; supply your own so retries are idempotent. |
| `name` | no | Display name, defaults to `task_id`. |
| `priority` | no | `-1000..1000`, higher runs first; ties break by submission order. |
| `map` | no | Disambiguates point ids defined on several maps. |
| `start_at` | no | Do not start before this time (`SCHEDULED` until then). ISO-8601 with offset, or epoch seconds. |
| `deadline_at` | no | Must be in the future and after `start_at`; a queued task that misses it fails without moving. |
| `repeat` | no | `count` 1..1000 and optional `interval_sec`; re-runs after success/partial. |
| `on_step_failure` | no | `continue` (default) or `abort`. |
| `callback_url` | no | `http(s)://`; overrides `--inspection-callback`. |
| `metadata` | no | Free-form object echoed back in every report. |
| `steps` | yes | 1..64 steps, unique `step_id`s. |

| Step field | Required | Notes |
|---|---|---|
| `step_id` | no | `[A-Za-z0-9][A-Za-z0-9._-]{0,31}`, defaults to `s<N>`; duplicate ids are rejected. |
| `target` | yes | `{"point_id": ...}` **or** `{"x": …, "y": …, "yaw": …}` (both rejected together). `floor` is optional. |
| `timeout_sec` | no | Navigation budget per attempt, 1..3600, default 180. |
| `retries` | no | 0..3 extra navigation attempts, default 0. |
| `dwell_sec` | no | Passed to `mission_manager`; actions start after the dwell. |
| `sensors` | no | `camera`, `lidar_scan` — declarative "measure here", deduplicated, run before `actions`. |
| `actions` | no | Up to 16 ordered actions, see below. |

A `point_id` is resolved from the point library; its `yaw` wins unless the step spells `yaw` out.

### Sensors and actions

| Name | Params | What it produces |
|---|---|---|
| sensor `camera` | – | One JPEG artifact. |
| sensor `lidar_scan` | – | One JSON artifact with `min/mean/max_range_m`, `front_min_range_m`, sample count, `frame_id`. |
| `capture_image` | `count` 1..30 | `count` JPEG artifacts. |
| `capture_burst` | `count` 1..30, `interval_sec` 0..5 | Time-lapse JPEGs. |
| `measure_scan` | `samples` 1..20, `interval_sec` 0..2 | Same metrics as the probe, for a single point measurement. |
| `speak` | `text` 1..200, `volume` 0..100 | Plays the text on the robot speaker. |
| `wait` | `sec` 0..600 | Cancellable pause (e.g. let a door close). |

Unknown keys anywhere in the payload are **rejected** rather than ignored, so a typo in the backend
fails loudly instead of silently doing nothing.

---

## Point library

```yaml
# inspection_points.yaml (default: <repo>/inspection_points.yaml)
schema_version: 1
points:
  lobby_door: {map: 1F, floor: "1F", x: 3.42, y: -1.08, yaw: 0.0, name: 大厅门口}
  pump_room:  {map: 1F, floor: "1F", x: 12.6, y: 4.35, yaw: 1.57, name: 泵房}
  # A mapping keyed by id or a list with explicit ids both work.
```

Points live in the map frame, in metres, `yaw` in radians. Keep the file under review in the same
repo as the maps so a re-survey updates both together. When several maps define the same id the
task's `map` field selects the right one, and an ambiguous id is a validation error.

---

## Reporting

Every task appends to `report.jsonl` and, unless callbacks are disabled, POSTs the same payload to
`callback_url` (or `--inspection-callback`) with `X-Inspection-Token` when a token is configured:

```json
{
  "schema_version": 1,
  "event": "FINISHED",
  "task_id": "night-1f-20261123",
  "name": "夜间巡检 · 一层",
  "state": "PARTIAL",
  "message": "one or more steps did not complete",
  "priority": 10,
  "map": "1F",
  "run_index": 1,
  "run_count": 2,
  "created_at": "2026-11-23T13:58:02+00:00",
  "started_at": "2026-11-23T14:00:01+00:00",
  "finished_at": "2026-11-23T14:06:34+00:00",
  "scheduled_for": "2026-11-23T14:00:00+00:00",
  "deadline_at": "2026-11-23T15:30:00+00:00",
  "progress": {"steps_total": 3, "steps_done": 2, "steps_failed": 1, "steps_skipped": 1, "current_step": 3},
  "steps": [
    {
      "step_id": "lobby",
      "state": "DONE",
      "message": "arrived and completed all sensors/actions",
      "attempts": 1,
      "arrived_at": 1763906401.2,
      "pose": {"x": 3.41, "y": -1.06, "yaw": 0.02, "yaw_deg": 1.1, "age_sec": 0.1},
      "sensors": [{"kind": "sensor", "name": "camera", "state": "OK", "duration_sec": 0.42,
                   "artifacts": [{"name": "lobby_sensor_camera_1763906401123_01.jpg", "size_bytes": 84213, "kind": "image"}],
                   "metrics": {"frame_stamp": 1763906401.1}}],
      "actions": [{"kind": "action", "name": "speak", "state": "OK", "metrics": {"text": "…", "volume": 80}}]
    }
  ],
  "artifacts": [{"name": "…jpg", "size_bytes": 84213, "created_at": 1763906401.1,
                 "url": "/api/inspection/tasks/night-1f-20261123/artifacts/…jpg"}],
  "robot": {"pose": {"x": 3.41, "y": -1.06, "yaw": 0.02}},
  "reported_at": 1763906794.5
}
```

Events: `ACCEPTED`, `STARTED`, `STEP` (one per finished step), `RESCHEDULED`, `FINISHED`,
`CANCELLED`, `FAILED`, `INTERRUPTED`.

Every event is POSTed by default (task accepts, step results, reschedule, terminal result); set
`report_progress: false` to receive only terminal results. Delivery is best effort: a backend outage
spools the payload to `pending_reports.jsonl` and retries in the background (5 attempts), so a dead
backend never blocks or slows the robot.

### Task and step states

| Task state | Meaning |
|---|---|
| `SCHEDULED` | Accepted, `start_at` is still in the future. |
| `QUEUED` | Waiting for the robot (only one task drives it at a time). |
| `RUNNING` | Owns the robot. |
| `SUCCEEDED` | Every step done. |
| `PARTIAL` | Finished, at least one step failed or was skipped. |
| `FAILED` | Nothing accomplished, deadline missed, dispatch rejected, or aborted before any success. |
| `CANCELLED` | Cancelled by the backend or an operator. |
| `INTERRUPTED` | The console restarted while the task was running. |

Step states: `PENDING`, `NAVIGATING`, `ACTING`, `DONE`, `PARTIAL` (arrived but a sensor/action
failed), `FAILED` (navigation failed after every retry), `SKIPPED` (`mission_manager` reported
`WAYPOINT_SKIPPED`), `CANCELLED`.

---

## HTTP API

All paths are under `/api/inspection`; every request needs `X-Inspection-Token` when a token is
configured.

| Method | Path | Purpose |
|---|---|---|
| `GET` | `/status` | Enabled flag, active task, queue head, counts, point-library status, storage free bytes, reporter state, last mission status. |
| `GET` | `/schema` | Machine-readable contract: states, limits, sensors, action params. |
| `GET` | `/points` | Flattened point library. |
| `GET` | `/tasks?state=&limit=` | Tasks, newest first (`limit` ≤ 200). |
| `GET` | `/tasks/<id>` | Full task record plus artifacts and the last report entry. |
| `GET` | `/tasks/<id>/report` | Append-only report history. |
| `GET` | `/tasks/<id>/artifacts` | Artifact list with download URLs. |
| `GET` | `/tasks/<id>/artifacts/<name>` | Raw artifact bytes (`image/jpeg`, `application/json`). |
| `POST` | `/tasks` | Submit one or more tasks. |
| `POST` | `/tasks/<id>/cancel` | Cancel a queued or running task. |

Errors: `400` invalid payload, `401` bad or missing token, `404` unknown task or path, `500` storage
failure. The console page (`/`) shows the same state in the **巡检任务** card, polls it from
`/api/state`, and can paste a task JSON for field debugging.

---

## Storage layout

```
~/ons2_inspection/
├── tasks/<task_id>/task.json          raw + normalized task (survives restart)
├── tasks/<task_id>/state.json         latest runtime snapshot
├── tasks/<task_id>/report.jsonl       report history
├── tasks/<task_id>/artifacts/*.jpg|*.json
└── pending_reports.jsonl              reports the backend did not accept
```

`max_tasks_kept` (default 200) prunes the oldest finished tasks at the end of each task, so an
unattended robot cannot fill the disk. A task interrupted by a restart is marked `INTERRUPTED`; a
`SCHEDULED`/`QUEUED` task keeps its place in the queue.

---

## Safety and arbitration

- **One owner at a time.** While an inspection task runs, `/api/mission/start` (operator waypoints) is
  refused with the running `task_id`. Operator missions are still allowed when only *queued* tasks
  exist; the queue waits.
- **E-stop wins.** A latched e-stop aborts the current step and fails the task; the task never
  releases it.
- **Chassis gate.** If the SW01 chassis bridge reports motion disabled, the task fails immediately
  instead of driving nowhere.
- **Stall alarm.** A stall detected during navigation cancels the mission and fails the step.
- **Timeouts.** Every step has a navigation budget (`timeout_sec`); `retries` re-issues the mission
  with a 1.5 s backoff.
- **Cancel is immediate.** The current mission is cancelled and remaining steps are never started;
  the step in flight is marked `CANCELLED`.

## Scope of this version

Implemented: scheduling (`start_at`, `deadline_at`, `priority`, repeats), point-library targets,
camera capture/burst, lidar metrics, TTS, waits, per-step retries/skip/abort, local evidence, HTTP
reports with spooling, console card.

Deliberately out of scope for now: bag recording or map alignment as an action, YOLO/vision
inference, outbound `http_call` actions (arbitrary URLs from a task), and inspection-specific
map/route editing. The action registry in `web_console/inspection/actions.py` is the extension point
for any of these.

## Simulator preview

`docs/multifloor/sim/mesh_nav.html` runs a task file written in this same schema in the browser, so a
task can be authored and watched before it reaches a robot: open
`mesh_nav.html#single,floor,insp`, or press **巡检示例** on a route that is already planned. Same
fields (`name`, `sensors`, `actions[].type`, `dwell_sec`), same vocabulary (`measure_scan`,
`capture_image`, `speak`, `wait`; sensors `camera` / `lidar_scan`), so nothing has to be translated
when the task moves from the page to the console. The dog parks at each station, `capture_image`
shoots a 640x480 JPEG through the dog's head-mounted Odin1 (the same 0.30 m mount and FOV
constants the synthetic scan uses); the panel row shows a 4:3 thumbnail and clicking it opens the
full frame in a window-sized overlay (`Esc` closes), `measure_scan` runs one occluded Odin1 raycast
and reports the front-sector nearest distance plus the lateral free width, and `speak` / `wait`
behave as they do on the robot. A badge pinned above the robot tracks it on screen and shows the
station it is on, the task state and progress, so the run can be followed on the model itself and
not only in the corner panel.

`target` is the only field the simulator extends: besides `point_id` / `x` / `y` it accepts where on
the route to stop — `{"from": "start" | "goal"}`, `{"route_at": 0..1}`, `{"waypoint": n}`,
`{"xyz": [x, y, z]}` — so one example file runs on any imported mesh. Replacing those with a
`point_id` or an `x`/`y` pair is the only edit needed before pushing the same file to the console.

The example task is [`multifloor/sim/inspection_task.example.json`](multifloor/sim/inspection_task.example.json);
behaviour, the panel and the headless probe are described in
[`multifloor/sim/README.md`](multifloor/sim/README.md#巡检任务示例-inspection-task-demo).

## Tests

```bash
python3 -m pytest web_console/tests/test_inspection.py -q
```

Covers schema limits and point resolution, storage safety, the executor (success, partial, retries,
deadline, schedule, cancel, abort, callback), operator arbitration, and every HTTP route including
token enforcement and artifact download.

# 巡检任务（接收巡检后台下发的结构化任务）

[English](inspection.md) | **中文**

巡检模块让**巡检后台**把结构化巡检任务推送到机器人：*什么时候跑、去哪些点位、用什么传感器、
执行什么操作*。机器人负责排队、用正常导航栈逐点到位、停稳后执行传感器与动作、把证据（图像、
雷达指标）存在本地，并把进度和结果回传。

它是现有导航栈的**客户端**，不是新的一层栈：

- 运动全部交给 `mission_manager`（`/waypoint_manager/start_mission`），**每个步骤下发一个单航点任务**。
  机器人只在航点之间移动，所有传感器/动作都在停稳后执行。
- 模块放在 `web_console/`（`web_console/inspection/`）里，因此同一套代码通过现有适配器同时跑在
  ROS 2（璇玑 SW01）和 ROS 1（Go2、Dobot Lumo）上，后台只需要访问控制台那一个 HTTP 端口。
- 没有新增任何 ROS 消息、服务或 C++ 改动。

---

## 执行模型

```
后台 ──POST /api/inspection/tasks──► 队列（落盘持久化）
                                        │
                           调度 ────────┴─ 按优先级，再按提交顺序；遵守 start_at / deadline_at
                                        │
                                        ▼
                  每个步骤：start_mission([waypoint]) → 等待 COMPLETED
                                        │
                                    机器人停稳
                                        │
                          传感器 → 动作 → 产物 + 指标
                                        │
                                        ▼
                       report.jsonl + POST callback_url（带 X-Inspection-Token）
```

为什么按步骤下发任务，而不是整任务一次下发：

- **重试与跳过**：某个点失败可以重试，仍失败则标 `SKIPPED`，不影响任务其余部分。
- **跨楼层安全**：`mission_manager` 的跨楼层换乘逻辑按航点生效，与操作员下发任务完全一致。
- **没有停留与动作的竞争**：`dwell_sec` 在 `mission_manager` 内结束，动作只在任务 `COMPLETED` 后开始。
- **取消可预期**：取消会停当前任务并放弃剩余步骤。

代价是每个步骤一次全局重规划，相对于巡检动作本身可以忽略不计。

---

## 启用方式

模块**默认开启**。用 `--no-inspection`、配置文件或 `ONS2_INSPECTION_ENABLE=0` 关闭。

```bash
# ROS 2（SW01）
python3 -m web_console.run --port 18090 --inspection-token "$ONS2_INSPECTION_TOKEN"

# ROS 1（Go2 / Dobot Lumo）——参数相同
ROS_VERSION=1 python3 -m web_console.run --port 18090 --inspection-token "$ONS2_INSPECTION_TOKEN"
```

systemd 单元（`scripts/systemd/web-console.service`）启动时没有带令牌，在把端口暴露到可信局域网之前，
请在单元里补上 `--inspection-token`（或一行 `Environment=`）。

| 参数 | 默认值 | 含义 |
|---|---|---|
| `--inspection` / `--no-inspection` | 开 | 是否接收任务并启动执行线程。 |
| `--inspection-dir` | `~/ons2_inspection` | 任务、产物、报告的存储根目录。 |
| `--inspection-points` | `<仓库>/inspection_points.yaml` | 点位库（见下文）。 |
| `--inspection-token` | 空（不鉴权） | 所有 `/api/inspection*` 请求必须携带的共享密钥。 |
| `--inspection-callback` | 空 | 任务自身没带回调地址时使用的默认上报地址。 |
| `--inspection-no-callback` | 开启回调 | 只存本地，不向后台 POST。 |
| `--inspection-callback-host` | 空（允许任意主机） | 回调主机白名单，可重复指定。 |
| `--inspection-config` | 无 | 带 `inspection:` 段的 YAML 配置文件。 |
| `--scan-topic` | `/scan_360` | `lidar_scan` 与 `measure_scan` 使用的 `sensor_msgs/LaserScan`。 |

环境变量：`ONS2_INSPECTION_ENABLE`、`ONS2_INSPECTION_DIR`、`ONS2_INSPECTION_POINTS`、
`ONS2_INSPECTION_TOKEN`、`ONS2_INSPECTION_CALLBACK`、`ONS2_INSPECTION_CALLBACKS`、
`ONS2_INSPECTION_CONFIG`。优先级：**命令行 > 环境变量 > 配置文件 > 默认值**。

```yaml
# inspection.yaml（用 --inspection-config 传入）
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

> **令牌很重要。** 不设 `--inspection-token` 时，任何能连上控制台端口的客户端都能投递任务，
> 也就是能开动机器人。只要机器人可能在不可信网络中被访问，就一定要设令牌。使用 `X-Inspection-Token`
> 请求头，或 `?token=` 查询参数（查询参数是为了让浏览器能直接打开任务详情链接）。

---

## 下发任务

```bash
curl -sS -X POST http://192.168.1.20:18090/api/inspection/tasks \
  -H 'Content-Type: application/json' \
  -H "X-Inspection-Token: $ONS2_INSPECTION_TOKEN" \
  -d @docs/inspection/task.example.json
```

支持的载荷形式：单个任务对象、任务数组、`{"tasks": [...]}` 或 `{"task": {...}}`（单次最多 50 个）。

```json
{
  "accepted": [{"task_id": "night-1f-20261123", "state": "QUEUED", "duplicate": false}],
  "rejected": [{"index": 1, "error": "steps[0].target needs a point_id or an x/y pair"}],
  "ok": true
}
```

- `200` 且 `rejected` 非空表示**部分成功**：列出的任务没进队列，其余已进入。
- `400` 且是同结构的响应体表示**一个都没接收**（`ok: false`），整批原样重试是安全的。
- 重复提交已知 `task_id` 会返回 `duplicate: true` 且不做任何改动，因此网络超时后重试是安全的。

### 任务格式

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

| 任务字段 | 必填 | 说明 |
|---|---|---|
| `task_id` | 否 | `[A-Za-z0-9][A-Za-z0-9._:-]{0,63}`。不填时自动生成（`insp_<毫秒>_<序号>`）；建议自己指定，便于重试幂等。 |
| `name` | 否 | 展示名，默认等于 `task_id`。 |
| `priority` | 否 | `-1000..1000`，大的先执行；相同则按提交顺序。 |
| `map` | 否 | 同一 id 在多个地图上定义时用于消歧。 |
| `start_at` | 否 | 此前不启动（状态为 `SCHEDULED`）。ISO-8601 带时区，或 epoch 秒。 |
| `deadline_at` | 否 | 必须晚于当前时间和 `start_at`；排队中过期则该任务不动机器人直接失败。 |
| `repeat` | 否 | `count` 1..1000 与可选 `interval_sec`；成功/部分成功后重新排期。 |
| `on_step_failure` | 否 | `continue`（默认）或 `abort`。 |
| `callback_url` | 否 | 必须是 `http(s)://`，覆盖 `--inspection-callback`。 |
| `metadata` | 否 | 任意对象，会在每次上报中原样带回。 |
| `steps` | 是 | 1..64 个步骤，`step_id` 不能重复。 |

| 步骤字段 | 必填 | 说明 |
|---|---|---|
| `step_id` | 否 | `[A-Za-z0-9][A-Za-z0-9._-]{0,31}`，默认 `s<N>`；重复会被拒绝。 |
| `target` | 是 | `{"point_id": ...}` **或** `{"x": …, "y": …, "yaw": …}`（两者同时出现会被拒绝）。`floor` 可选。 |
| `timeout_sec` | 否 | 每次导航的时间预算 1..3600，默认 180。 |
| `retries` | 否 | 额外重试次数 0..3，默认 0。 |
| `dwell_sec` | 否 | 透传给 `mission_manager`；动作在停留结束后开始。 |
| `sensors` | 否 | `camera`、`lidar_scan`，声明「到此需采集」，去重后先于 `actions` 执行。 |
| `actions` | 否 | 最多 16 个有序动作，见下表。 |

`point_id` 从点位库解析；除非步骤里显式写了 `yaw`，否则用点位库的 `yaw`。

### 传感器与动作

| 名称 | 参数 | 产物 |
|---|---|---|
| 传感器 `camera` | – | 一张 JPEG 产物。 |
| 传感器 `lidar_scan` | – | 一份 JSON，含 `min/mean/max_range_m`、`front_min_range_m`、采样次数、`frame_id`。 |
| `capture_image` | `count` 1..30 | `count` 张 JPEG。 |
| `capture_burst` | `count` 1..30、`interval_sec` 0..5 | 定时连拍。 |
| `measure_scan` | `samples` 1..20、`interval_sec` 0..2 | 与雷达探针相同的指标，可按点单独测量。 |
| `speak` | `text` 1..200、`volume` 0..100 | 通过机器人扬声器播报。 |
| `wait` | `sec` 0..600 | 可取消的等待（例如等门关闭）。 |

载荷里任何位置的未知字段都会被**拒绝**而不是忽略，因此后台的拼写错误会立刻报错，而不是静默失效。

---

## 点位库

```yaml
# inspection_points.yaml（默认路径：<仓库>/inspection_points.yaml）
schema_version: 1
points:
  lobby_door: {map: 1F, floor: "1F", x: 3.42, y: -1.08, yaw: 0.0, name: 大厅门口}
  pump_room:  {map: 1F, floor: "1F", x: 12.6, y: 4.35, yaw: 1.57, name: 泵房}
  # 既支持以 id 为键的映射，也支持带 id 字段的列表。
```

坐标在地图坐标系，单位米，`yaw` 单位弧度。点位库建议和地图放在同一个仓库里一起评审，重新建图时同步更新。
同一 id 在多个地图上定义时，用任务的 `map` 字段选择；仍然歧义则视为校验错误。

---

## 上报

每个任务都会追加到 `report.jsonl`，并在开启回调时把同样的载荷 POST 到 `callback_url`（或
`--inspection-callback`），配置了令牌时带 `X-Inspection-Token`：

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

事件：`ACCEPTED`、`STARTED`、`STEP`（每个步骤一条）、`RESCHEDULED`、`FINISHED`、`CANCELLED`、
`FAILED`、`INTERRUPTED`。

默认每个事件都会 POST（任务接收、步骤结果、重新排期、最终结果）；设 `report_progress: false`
只接收最终结果。投递是尽力而为：后台不可用时载荷写入 `pending_reports.jsonl`，后台线程稍后重试
（最多 5 次），因此后台挂掉既不会阻塞也不会拖慢机器人。

### 任务与步骤状态

| 任务状态 | 含义 |
|---|---|
| `SCHEDULED` | 已接收，`start_at` 还没到。 |
| `QUEUED` | 等待机器人空闲（同一时刻只有一个任务占用机器人）。 |
| `RUNNING` | 正在占用机器人。 |
| `SUCCEEDED` | 所有步骤完成。 |
| `PARTIAL` | 已结束，但至少有一个步骤失败或被跳过。 |
| `FAILED` | 一无所获：全部失败、超过 deadline、下发被拒、或在任何成功之前中止。 |
| `CANCELLED` | 被后台或操作员取消。 |
| `INTERRUPTED` | 任务执行期间控制台重启。 |

步骤状态：`PENDING`、`NAVIGATING`、`ACTING`、`DONE`、`PARTIAL`（到位但某个传感器/动作失败）、
`FAILED`（重试后仍导航失败）、`SKIPPED`（`mission_manager` 上报 `WAYPOINT_SKIPPED`）、`CANCELLED`。

---

## HTTP 接口

所有路径都在 `/api/inspection` 下；配置了令牌时每个请求都要带 `X-Inspection-Token`。

| 方法 | 路径 | 用途 |
|---|---|---|
| `GET` | `/status` | 是否启用、当前任务、队首、计数、点位库状态、存储剩余空间、上报器状态、最近一次导航状态。 |
| `GET` | `/schema` | 机器可读的契约：状态、上限、传感器、动作参数。 |
| `GET` | `/points` | 扁平化后的点位库。 |
| `GET` | `/tasks?state=&limit=` | 任务列表，新的在前（`limit` ≤ 200）。 |
| `GET` | `/tasks/<id>` | 完整任务记录，附产物列表与最后一条上报。 |
| `GET` | `/tasks/<id>/report` | 追加式上报历史。 |
| `GET` | `/tasks/<id>/artifacts` | 产物列表及下载地址。 |
| `GET` | `/tasks/<id>/artifacts/<name>` | 产物原始字节（`image/jpeg`、`application/json`）。 |
| `POST` | `/tasks` | 下发一个或多个任务。 |
| `POST` | `/tasks/<id>/cancel` | 取消排队中或执行中的任务。 |

错误码：`400` 载荷非法、`401` 令牌错误或缺失、`404` 任务或路径不存在、`500` 存储失败。控制台页面（`/`）
的 **巡检任务** 卡片展示同一份状态（从 `/api/state` 轮询），并可粘贴任务 JSON 做现场调试。

---

## 存储结构

```
~/ons2_inspection/
├── tasks/<task_id>/task.json          原始 + 归一化后的任务（重启后仍可恢复）
├── tasks/<task_id>/state.json         最近一次运行快照
├── tasks/<task_id>/report.jsonl       上报历史
├── tasks/<task_id>/artifacts/*.jpg|*.json
└── pending_reports.jsonl              后台拒收的上报
```

`max_tasks_kept`（默认 200）会在每个任务结束后清理最旧的已完成任务，无人值守时不会把磁盘写满。
重启前正在执行的任务会标为 `INTERRUPTED`；`SCHEDULED`/`QUEUED` 的任务保留在原队列位置。

---

## 安全与仲裁

- **同一时刻只有一个占用者。** 巡检任务执行期间，`/api/mission/start`（操作员航点）会被拒绝并返回
  正在执行的 `task_id`。只有*排队中*任务时操作员仍可下发，队列会等待。
- **急停优先。** 锁存急停会中止当前步骤并让任务失败；任务不会替你解除急停。
- **底盘门控。** SW01 底盘桥上报运动被禁止时，任务立即失败而不是空跑。
- **卡死告警。** 导航期间触发卡死告警会取消当前任务并判该步骤失败。
- **超时。** 每个步骤有导航预算（`timeout_sec`）；`retries` 会在 1.5 s 退避后重新下发任务。
- **取消即时生效。** 取消当前任务，剩余步骤不再启动，进行中的步骤标为 `CANCELLED`。

## 本版本范围

已实现：调度（`start_at`、`deadline_at`、`priority`、重复）、点位库目标、拍照/连拍、雷达指标、
TTS 播报、等待、按步骤重试/跳过/中止、本地证据、带脱机缓存的 HTTP 上报、控制台卡片。

本版本明确不做：把录包或地图对齐做成动作、YOLO/视觉推理、出站 `http_call`（任务里指定任意 URL）、
巡检专用的地图/路线编辑。`web_console/inspection/actions.py` 的动作注册表就是这些能力的扩展点。

## 仿真里先看效果

`docs/multifloor/sim/mesh_nav.html` 用同一套 schema 在浏览器里跑任务文件，可以先写任务、先看效果，
再上实机：打开 `mesh_nav.html#single,floor,insp`，或在已规划的路线旁点「巡检示例」。字段相同
（`name`、`sensors`、`actions[].type`、`dwell_sec`），词表相同（`measure_scan`、`capture_image`、
`speak`、`wait`，传感器 `camera` / `lidar_scan`），任务从页面搬到控制台不需要改字段名。狗到站停车，
`capture_image` 用狗头上那台 Odin1 的第一视角（与合成传感同一个 0.30 m 安装位与视场角）拍一张
640×480 的 JPEG，面板里是 4:3 缩略图、点一下铺满窗口看整帧（`Esc` 关闭），`measure_scan` 用页面自带
的 Odin1 模型真的打一次有遮挡的射线、报前向扇区最近距离与横向净空，`speak` / `wait` 与实机一致。
狗身上方还挂着一块浮标，跟着狗的画面位置走，写着当前站点、任务状态与进度条——直接在模型上就能
跟着看，不必只盯右上角的面板。

`target` 是仿真唯一扩展的字段：除了 `point_id` / `x` / `y`，还接受「停在线路的哪里」——
`{"from": "start" | "goal"}`、`{"route_at": 0..1}`、`{"waypoint": n}`、`{"xyz": [x, y, z]}`，
所以一份示例文件能在任意导入的网格上跑。上实机前把它们换成 `point_id` 或 `x`/`y` 即可，其余不动。

示例任务见 [`multifloor/sim/inspection_task.example.json`](multifloor/sim/inspection_task.example.json)；
行为、面板与无头探针见 [`multifloor/sim/README.md`](multifloor/sim/README.md#巡检任务示例-inspection-task-demo)。

## 测试

```bash
python3 -m pytest web_console/tests/test_inspection.py -q
```

覆盖：格式与上限校验、点位解析、存储安全、执行器（成功、部分、重试、deadline、定时、取消、中止、
回调）、操作员仲裁，以及全部 HTTP 路由（含令牌鉴权与产物下载）。

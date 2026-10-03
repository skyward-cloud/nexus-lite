# `/px4_bridge/in/task_cmd` 使用说明

外部通过该话题向 `px4_bridge` 下发一次性任务（解锁、切模式、指点等）。  
消息类型：`px4_bridge_msgs/msg/TaskCommand`。

状态回读请订阅：`/px4_bridge/out/status`（`BridgeStatus`）。  
高频速度/姿态请用：`/px4_bridge/in/realtime_control`（本文不展开）。

---

## 1. 消息字段

任务参数为**扁平键值字段**。

### 元数据

| 字段 | 类型 | 说明 |
|------|------|------|
| `task_id` | `string` | 任务 ID，建议唯一（如 `arm-1`） |
| `task_type` | `string` | 任务类型，见下文枚举 |
| `deadline_ms` | `int32` | 超时（毫秒）。`0` 表示使用桥接默认值 |

### 任务参数（按 `task_type` 使用）

| 字段 | 类型 | 用于 |
|------|------|------|
| `arm` | `bool` | ARMING |
| `confirm_current_mode` | `bool` | ARMING（可选） |
| `mode` | `string` | MODE_SWITCH |
| `x`,`y`,`z` | `float32` | POSITION_CONTROL |
| `vx`,`vy`,`vz` | `float32` | VELOCITY_CONTROL |
| `yaw` | `float32` | 位置 / 速度 |
| `arrival_radius_m` / `arrival_dz_m` / `arrival_speed_mps` | `float32` | POSITION 到点（`0` 表示用桥接默认） |

桥接侧行为：**队列只保留最新一条**任务；非法任务会打日志并忽略，不拖垮节点。

---

## 2. 坐标系

- 位置 / 速度：局部 **ENU**（东-北-天），**z 向上为正**
- 偏航 `yaw`：ENU 下绕天向，单位 **rad**

起飞到 2 m 示例：`z: 2.0`。

---

## 3. 支持的 `task_type`

| task_type | 必填字段 | 默认 deadline_ms | 说明 |
|-----------|----------|------------------|------|
| `ARMING` | `arm` | 5000 | 解锁 / 上锁 |
| `MODE_SWITCH` | `mode` | 5000 | 切模式；`RETURN_HOME` 走返航流程 |
| `POSITION_CONTROL` | `x`,`y`,`z` | 1800000 | 指点，直到到点或超时 |
| `VELOCITY_CONTROL` | `vx`,`vy`,`vz` | 10000 | 速度控制，持续到 deadline |
| `KILL_SWITCH` | （无） | 5000 | 停桨|

姿态/推力持续控制请用 `/px4_bridge/in/realtime_control`。

---

## 4. 各类型样例

### 4.1 `ARMING`

| 字段 | 必填 | 说明 |
|------|------|------|
| `arm` | 是 | `true` 解锁，`false` 上锁 |
| `confirm_current_mode` | 否 | `true` 时跳过「先切 POSCTL」 |

**解锁：**

```bash
ros2 topic pub --once /px4_bridge/in/task_cmd px4_bridge_msgs/msg/TaskCommand "{
  task_id: 'arm-1',
  task_type: 'ARMING',
  arm: true
}"
```

**上锁：**

```bash
ros2 topic pub --once /px4_bridge/in/task_cmd px4_bridge_msgs/msg/TaskCommand "{
  task_id: 'disarm-1',
  task_type: 'ARMING',
  arm: false
}"
```

**Python：**

```python
from px4_bridge_msgs.msg import TaskCommand

msg = TaskCommand()
msg.task_id = "arm-1"
msg.task_type = "ARMING"
msg.arm = True
# pub.publish(msg)
```

或使用 `examples/common.py` 的 `BridgeClient.submit_task("arm-1", "ARMING", {"arm": True})`。

---

### 4.2 `MODE_SWITCH`

允许的 `mode`：`OFFBOARD` · `LAND` · `HOLD` · `RETURN_HOME` · `MANUAL` · `ALTCTL` / `ALTITUDE` · `POSCTL` / `POSITION` · `AUTO` · `ACRO` · `STABILIZED` / `STAB` · `RATTITUDE`

**降落：**

```bash
ros2 topic pub --once /px4_bridge/in/task_cmd px4_bridge_msgs/msg/TaskCommand "{
  task_id: 'land-1',
  task_type: 'MODE_SWITCH',
  mode: 'LAND'
}"
```

---

### 4.3 `POSITION_CONTROL`

```bash
ros2 topic pub --once /px4_bridge/in/task_cmd px4_bridge_msgs/msg/TaskCommand "{
  task_id: 'takeoff-1',
  task_type: 'POSITION_CONTROL',
  deadline_ms: 60000,
  x: 0.0,
  y: 0.0,
  z: 2.0,
  yaw: 0.0,
  arrival_radius_m: 0.3,
  arrival_dz_m: 0.2,
  arrival_speed_mps: 0.5
}"
```

到点默认（字段为 0 时）：水平半径 `1.0` m，高度容差 `0.6` m，速度阈值 `0.5` m/s。

---

### 4.4 `VELOCITY_CONTROL`

```bash
ros2 topic pub --once /px4_bridge/in/task_cmd px4_bridge_msgs/msg/TaskCommand "{
  task_id: 'vel-1',
  task_type: 'VELOCITY_CONTROL',
  deadline_ms: 3000,
  vx: 0.5,
  vy: 0.0,
  vz: 0.0,
  yaw: 0.0
}"
```

长时间遥控建议用 `/px4_bridge/in/realtime_control`。

---

### 4.5 `KILL_SWITCH`

```bash
ros2 topic pub --once /px4_bridge/in/task_cmd px4_bridge_msgs/msg/TaskCommand "{
  task_id: 'kill-1',
  task_type: 'KILL_SWITCH'
}"
```

---

## 5. 推荐流程

1. 确认 `/px4_bridge/out/status` 中 `connected=true`
2. `ARMING` + `arm: true`
3. `POSITION_CONTROL` 到起飞高度
4. 再发指点 `POSITION_CONTROL`
5. `MODE_SWITCH` + `mode: LAND`
6. `ARMING` + `arm: false`

```bash
source install/setup.bash
python3 examples/01_arm_takeoff_land.py
python3 examples/02_position_point.py --x 2 --y 0 --z 1.5
python3 examples/04_mode_switch.py --to POSCTL
python3 examples/05_keyboard_teleop.py
```

---

## 6. 状态回读

```bash
ros2 topic echo /px4_bridge/out/status
```

| 字段 | 含义 |
|------|------|
| `connected` / `armed` / `mode` | 飞控连接、解锁、模式 |
| `lifecycle_state` | `READY` / `EXECUTING` / `FAULT` 等 |
| `active_task_id` | 当前任务 |
| `error` | 当前错误；非 `FAULT` 时为空 |
| `position` / `velocity` | ENU |
| `rth_phase` / `rth_detail` | 返航进度 |

---

## 7. 常见错误

| 现象 | 原因 | 处理 |
|------|------|------|
| `payload 缺少字段` | 未填 `arm` / `mode` / `x,y,z` 等 | 按上表补齐对应字段 |
| `mode 不支持` | 模式名错误 | 使用允许列表 |
| `飞控未连接，拒绝解锁` | 尚无 PX4 数据 | 检查 Agent 与 `ROS_DOMAIN_ID` |
| 位置任务不结束 | 未到点或坐标系搞反 | 确认 ENU；放宽 `arrival_*` |

---

## 8. 与 realtime 的分工

| 场景 | 话题 |
|------|------|
| 解锁、降落、返航、单次指点 | `/px4_bridge/in/task_cmd` |
| 持续速度/位置/姿态遥操 | `/px4_bridge/in/realtime_control` |

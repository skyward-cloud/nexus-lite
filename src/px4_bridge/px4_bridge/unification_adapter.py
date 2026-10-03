"""PX4 Topic 读写适配层（lite：仅 ROS，无避障/DDS）。"""

from __future__ import annotations

import math
import time
from typing import Any, Callable, Iterable, Optional

from .models import DroneSnapshot, TaskCommand, TaskType
from .failsafe import (
    DEFAULT_FAILSAFE_FLAG_SKIP,
    failsafe_state_from_failsafe_flags,
    load_failsafe_flag_skip_from_config_file,
)
from .util import (
    convert_yaw,
    enu_to_ned,
    ned_to_enu,
    transform_body_vec,
    transform_world_vec,
)


def euler_from_quat(
    qw: float, qx: float, qy: float, qz: float
) -> tuple[float, float, float]:
    """从四元数计算欧拉角。"""
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    pitch = math.asin(2.0 * (qw * qy - qz * qx))
    roll = math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
    return roll, pitch, yaw


def wrap_pi(angle: float) -> float:
    """将角度归一化到 [-pi, pi]。"""
    return math.atan2(math.sin(angle), math.cos(angle))


def _home_position_valid_from_msg(msg: Any) -> bool:
    """PX4 HomePosition 至少一项有效才视为可用。"""
    valid_lpos = bool(getattr(msg, "valid_lpos", False))
    valid_hpos = bool(getattr(msg, "valid_hpos", False))
    valid_alt = bool(getattr(msg, "valid_alt", False))
    return valid_lpos or valid_hpos or valid_alt


try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from px4_msgs.msg import (
        BatteryStatus,
        FailsafeFlags,
        HomePosition,
        OffboardControlMode,
        TrajectorySetpoint,
        VehicleAttitude,
        VehicleAttitudeSetpoint,
        VehicleCommand,
        VehicleCommandAck,
        VehicleGlobalPosition,
        VehicleLocalPosition,
        VehicleStatus,
    )

    NAVIGATION_STATE_POSCTL = int(getattr(VehicleStatus, "NAVIGATION_STATE_POSCTL", 2))
    NAVIGATION_STATE_OFFBOARD = int(
        getattr(VehicleStatus, "NAVIGATION_STATE_OFFBOARD", 14)
    )
    ROS2_AVAILABLE = True
except ImportError:
    NAVIGATION_STATE_POSCTL = 2
    NAVIGATION_STATE_OFFBOARD = 14
    ROS2_AVAILABLE = False
    rclpy = None  # type: ignore[assignment]
    Node = Any
    VehicleGlobalPosition = None  # type: ignore[misc, assignment]

if ROS2_AVAILABLE:
    _NAV_STATE_NAME_BY_VALUE: dict[int, str] = {}
    _NAV_STATE_VALUE_BY_NAME: dict[str, int] = {}
    for _name in dir(VehicleStatus):
        if not _name.startswith("NAVIGATION_STATE_"):
            continue
        try:
            _value = int(getattr(VehicleStatus, _name))
        except (TypeError, ValueError):
            continue
        short = _name.removeprefix("NAVIGATION_STATE_")
        _NAV_STATE_NAME_BY_VALUE[_value] = short
        _NAV_STATE_VALUE_BY_NAME[short] = _value
else:
    _NAV_STATE_NAME_BY_VALUE = {}
    _NAV_STATE_VALUE_BY_NAME = {}


class PX4TopicsAdapter:
    """封装 PX4 状态订阅与控制发布。"""

    def __init__(
        self,
        node: Optional[Node] = None,
        *,
        failsafe_flag_skip: Optional[Iterable[str]] = None,
    ) -> None:
        """初始化适配器并绑定 PX4 话题（共用进程内单一 ROS Node）。"""
        self._node: Optional[Node] = node
        self._snapshot = DroneSnapshot()
        self.is_ready = False
        self._topic_rx_ms: dict[str, int] = {}
        self._last_command_ack: dict[int, tuple[int, int]] = {}
        self._last_vehicle_command_sent: Optional[int] = None
        self._home_from_arm = False
        self.pub_offboard_mode = None
        self.pub_trajectory = None
        self.pub_attitude_sp = None
        self.pub_vehicle_cmd = None
        # 悬停锁存位姿（ENU: x, y, z, yaw）；None 表示下次悬停时重新捕获
        self._hover_setpoint: Optional[tuple[float, float, float, float]] = None

        if failsafe_flag_skip is not None:
            self._failsafe_flag_skip = frozenset(failsafe_flag_skip)
        else:
            loaded = load_failsafe_flag_skip_from_config_file()
            self._failsafe_flag_skip = frozenset(
                loaded if loaded else DEFAULT_FAILSAFE_FLAG_SKIP
            )

        if not ROS2_AVAILABLE or self._node is None:
            return

        self.qos_depth = 10
        # 与 PX4 uXRCE-DDS / microdds 默认一致：BEST_EFFORT + TRANSIENT_LOCAL；
        # 使用默认 RELIABLE 订阅会导致 QoS 不匹配、收不到消息。
        self.px4_out_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=self.qos_depth,
        )
        self._bind()

    def _bind(self) -> None:
        """创建 PX4 订阅与发布者。"""
        if self._node is None:
            return
        self._node.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status",
            self._on_vehicle_status,
            self.px4_out_qos,
        )
        self._node.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self._on_vehicle_local_position,
            self.px4_out_qos,
        )
        if VehicleGlobalPosition is not None:
            self._node.create_subscription(
                VehicleGlobalPosition,
                "/fmu/out/vehicle_global_position",
                self._on_vehicle_global_position,
                self.px4_out_qos,
            )
        self._node.create_subscription(
            VehicleAttitude,
            "/fmu/out/vehicle_attitude",
            self._on_vehicle_attitude,
            self.px4_out_qos,
        )
        self._node.create_subscription(
            BatteryStatus,
            "/fmu/out/battery_status",
            self._on_battery_status,
            self.px4_out_qos,
        )
        self._node.create_subscription(
            FailsafeFlags,
            "/fmu/out/failsafe_flags",
            self._on_failsafe_flags,
            self.px4_out_qos,
        )
        self._node.create_subscription(
            HomePosition,
            "/fmu/out/home_position",
            self._on_home_position,
            self.px4_out_qos,
        )
        self._node.create_subscription(
            VehicleCommandAck,
            "/fmu/out/vehicle_command_ack",
            self._on_vehicle_command_ack,
            self.px4_out_qos,
        )

        self.pub_offboard_mode = self._node.create_publisher(
            OffboardControlMode,
            "/fmu/in/offboard_control_mode",
            self.qos_depth,
        )
        self.pub_trajectory = self._node.create_publisher(
            TrajectorySetpoint,
            "/fmu/in/trajectory_setpoint",
            self.qos_depth,
        )
        self.pub_attitude_sp = self._node.create_publisher(
            VehicleAttitudeSetpoint,
            "/fmu/in/vehicle_attitude_setpoint",
            self.qos_depth,
        )
        self.pub_vehicle_cmd = self._node.create_publisher(
            VehicleCommand,
            "/fmu/in/vehicle_command",
            self.qos_depth,
        )
        self.is_ready = True

    def spin_once(self) -> None:
        """执行一次 ROS2 回调处理。"""
        if self.is_ready and self._node is not None:
            try:
                rclpy.spin_once(self._node, timeout_sec=0.0)
            except IndexError:
                # ROS2 Humble 在 QoS event 重建时偶发 wait set 越界。
                pass
        else:
            raise RuntimeError("Client not ready")

    def close(self) -> None:
        """解除绑定（不销毁共享 Node，由 PX4BridgeNode 负责）。"""
        self._node = None
        self.pub_offboard_mode = None
        self.pub_trajectory = None
        self.pub_attitude_sp = None
        self.pub_vehicle_cmd = None
        self.is_ready = False

    def get_snapshot(self) -> DroneSnapshot:
        """获取最新无人机状态快照。"""
        return self._snapshot

    def _mark_topic(self, name: str) -> None:
        self._topic_rx_ms[name] = int(time.time() * 1000)

    def telemetry_debug(self, now_ms: int) -> dict[str, Any]:
        """供 HTTP /status 输出：ROS2 是否就绪、各话题距上次接收的间隔（ms）。"""
        if not self.is_ready:
            return {
                "client_active": False,
                "hint": "未初始化 PX4 订阅（检查是否安装 Fast-DDS、px4_msgs）",
            }
        ages: dict[str, Optional[int]] = {}
        for key, ts in self._topic_rx_ms.items():
            ages[key] = now_ms - ts if ts else None
        stale = [k for k, a in ages.items() if a is None or a > 2000]
        hint: Optional[str] = None
        if ages.get("vehicle_status") is None:
            hint = (
                "未收到 /fmu/out/vehicle_status：请确认 PX4 SITL/真机已启动，"
                "Micro-XRCE-DDS-Agent 与桥接使用相同 ROS_DOMAIN_ID，且 uXRCE 已连上。"
            )
        elif stale:
            hint = f"下列话题超过 2s 无更新（可能 EKF 未就绪或话题停发）: {', '.join(stale)}"
        return {
            "client_active": True,
            "topic_age_ms": ages,
            "hint": hint,
        }

    def publish_default_offboard(
        self,
        position: bool = False,
        velocity: bool = False,
        attitude: bool = False,
        acceleration: bool = False,
    ) -> None:
        """发布 offboard 控制模式。"""
        msg = OffboardControlMode()
        msg.timestamp = self._timestamp_us()
        msg.position = position
        msg.velocity = velocity or acceleration
        msg.acceleration = acceleration
        msg.attitude = attitude
        msg.body_rate = False
        self.pub_offboard_mode.publish(msg)

    def _on_vehicle_status(self, msg: Any) -> None:
        """处理飞控状态。"""
        self._mark_topic("vehicle_status")
        nav_state = int(getattr(msg, "nav_state", -1))
        arming_state = int(getattr(msg, "arming_state", 0))
        self._snapshot.connected = True
        self._snapshot.mode = _NAV_STATE_NAME_BY_VALUE.get(
            nav_state, f"UNKNOWN({nav_state})"
        )
        self._snapshot.armed = arming_state == 2
        self._snapshot.updated_at_ms = int(time.time() * 1000)

    def _on_vehicle_local_position(self, msg: Any) -> None:
        """处理位置与速度（PX4 NED → 内部 ENU）。"""
        self._mark_topic("vehicle_local_position")
        self._snapshot.local_position_xy_valid = bool(getattr(msg, "xy_valid", False))
        self._snapshot.local_position_z_valid = bool(getattr(msg, "z_valid", False))
        self._snapshot.local_velocity_xy_valid = bool(getattr(msg, "v_xy_valid", False))
        self._snapshot.local_velocity_z_valid = bool(getattr(msg, "v_z_valid", False))
        pos_ned = (
            float(getattr(msg, "x", 0.0)),
            float(getattr(msg, "y", 0.0)),
            float(getattr(msg, "z", 0.0)),
        )
        vel_ned = (
            float(getattr(msg, "vx", 0.0)),
            float(getattr(msg, "vy", 0.0)),
            float(getattr(msg, "vz", 0.0)),
        )
        self._snapshot.position = transform_world_vec(pos_ned)
        self._snapshot.velocity = transform_world_vec(vel_ned)
        self._snapshot.updated_at_ms = int(time.time() * 1000)

    def _on_vehicle_global_position(self, msg: Any) -> None:
        """处理全局位置（WGS84）。PX4 通常为弧度；若量级像度则不再换算。"""
        self._mark_topic("vehicle_global_position")
        lat_raw = float(getattr(msg, "lat", 0.0))
        lon_raw = float(getattr(msg, "lon", 0.0))
        # VehicleGlobalPosition.lat/lon 为弧度（约 ±1.57 / ±3.14）；若已为度则 |lat| 常 > 3。
        if abs(lat_raw) <= math.pi / 2 + 1e-3 and abs(lon_raw) <= math.pi * 2 + 1e-3:
            lat_deg = math.degrees(lat_raw)
            lon_deg = math.degrees(lon_raw)
        else:
            lat_deg = lat_raw
            lon_deg = lon_raw
        self._snapshot.global_lat_deg = lat_deg
        self._snapshot.global_lon_deg = lon_deg
        self._snapshot.global_position_valid = True
        self._snapshot.updated_at_ms = int(time.time() * 1000)

    def _on_vehicle_attitude(self, msg: Any) -> None:
        """处理姿态四元数（PX4 NED/FRD → 内部 ENU/FLU）。"""
        self._mark_topic("vehicle_attitude")
        q = getattr(msg, "q", [1.0, 0.0, 0.0, 0.0])
        quat_ned = (float(q[0]), float(q[1]), float(q[2]), float(q[3]))
        _pos, quat_enu = ned_to_enu((0.0, 0.0, 0.0), quat_ned)
        self._snapshot.attitude_quat = (
            float(quat_enu[0]),
            float(quat_enu[1]),
            float(quat_enu[2]),
            float(quat_enu[3]),
        )
        self._snapshot.updated_at_ms = int(time.time() * 1000)

    def _on_battery_status(self, msg: Any) -> None:
        """处理电量信息。"""
        self._mark_topic("battery_status")
        self._snapshot.battery_remaining = float(getattr(msg, "remaining", 1.0))
        self._snapshot.battery_voltage = float(getattr(msg, "voltage_v", 0.0))
        self._snapshot.updated_at_ms = int(time.time() * 1000)

    def _on_failsafe_flags(self, msg: Any) -> None:
        """处理 failsafe 标志。"""
        self._mark_topic("failsafe_flags")
        active, reason = failsafe_state_from_failsafe_flags(
            msg, self._failsafe_flag_skip
        )
        self._snapshot.failsafe = active
        self._snapshot.failsafe_reason = reason if active else ""
        self._snapshot.updated_at_ms = int(time.time() * 1000)

    def _on_home_position(self, msg: Any) -> None:
        """处理 home 点（PX4 NED → 内部 ENU；不覆盖解锁时记录的 home）。"""
        self._mark_topic("home_position")
        if self._home_from_arm and self._snapshot.home_valid:
            return
        if not _home_position_valid_from_msg(msg):
            return
        home_ned = (
            float(getattr(msg, "x", 0.0)),
            float(getattr(msg, "y", 0.0)),
            float(getattr(msg, "z", 0.0)),
        )
        self._snapshot.home_position = transform_world_vec(home_ned)
        self._snapshot.home_lat_deg = float(
            getattr(msg, "lat", getattr(msg, "latitude_deg", 0.0))
        )
        self._snapshot.home_lon_deg = float(
            getattr(msg, "lon", getattr(msg, "longitude_deg", 0.0))
        )
        self._snapshot.home_alt_msl_m = float(
            getattr(
                msg,
                "alt",
                getattr(msg, "altitude_msl_m", getattr(msg, "altitude_msl", 0.0)),
            )
        )
        self._snapshot.home_valid = True
        self._snapshot.home_updated_at_ms = int(time.time() * 1000)
        self._snapshot.updated_at_ms = self._snapshot.home_updated_at_ms

    def _on_vehicle_command_ack(self, msg: Any) -> None:
        """处理 vehicle command ACK。"""
        command = int(getattr(msg, "command", -1))
        result = int(getattr(msg, "result", -1))
        self._last_command_ack[command] = (result, int(time.time() * 1000))

    def current_nav_state(self) -> int:
        """从快照解析 `vehicle_status.nav_state`，未知时返回 -1。"""
        m = str(self._snapshot.mode).strip()
        if not m:
            return -1
        # 统一仅接受 VehicleStatus 常量派生的模式名（如 POSCTL / OFFBOARD / AUTO_RTL）。
        return _NAV_STATE_VALUE_BY_NAME.get(m, -1)

    def is_navigation_state_posctl(self) -> bool:
        """当前导航状态是否为定点模式（POSCTL）。"""
        return self.current_nav_state() == NAVIGATION_STATE_POSCTL

    def wait_nav_state(self, expected: int, timeout_ms: int = 4000) -> tuple[bool, str]:
        """轮询直到 nav_state 等于 expected 或超时。"""
        start = int(time.time() * 1000)
        while int(time.time() * 1000) - start <= timeout_ms:
            if self.current_nav_state() == expected:
                return True, "OK"
            self.spin_once()
            time.sleep(0.02)
        return False, (
            f"nav_state 未在 {timeout_ms}ms 内变为 {expected}，当前={self.current_nav_state()}"
        )

    def switch_to_posctl_and_confirm(
        self,
        timeout_ack_ms: int = 3000,
        timeout_nav_ms: int = 8000,
    ) -> None:
        """发送切入定点模式并等待 ACK 与 nav_state 进入 POSCTL。

        若当前为 OFFBOARD，先短时预发 offboard 流再切模式（与切入 OFFBOARD 前类似），
        避免无链路时切 POSCTL 无应答；不在 LAND/AUTO_LAND 过程中抢切定点（由调用方保证时机）。
        """
        self.spin_once()
        if self.current_nav_state() == NAVIGATION_STATE_OFFBOARD:
            self._prime_offboard_stream(duration_s=0.5, rate_hz=20.0)
        task = TaskCommand(
            task_id="internal-arm-posctl",
            task_type=TaskType.MODE_SWITCH,
            payload={"mode": "POSCTL"},
        )
        self.publish_task(task)
        cmd = self.last_vehicle_command_sent()
        if cmd is None:
            raise RuntimeError("定点模式命令未发送")
        ack_ok, ack_detail = self.wait_vehicle_command_ack(
            cmd, timeout_ms=timeout_ack_ms
        )
        if not ack_ok:
            raise RuntimeError(f"切入定点模式 ACK 失败: {ack_detail}")
        nav_ok, nav_detail = self.wait_nav_state(
            NAVIGATION_STATE_POSCTL, timeout_ms=timeout_nav_ms
        )
        if not nav_ok:
            raise RuntimeError(f"切入定点模式后状态未就绪: {nav_detail}")

    def switch_to_offboard_and_confirm(
        self,
        timeout_ack_ms: int = 3000,
        timeout_nav_ms: int = 10000,
    ) -> None:
        """切入 OFFBOARD 并等待 nav_state。

        TrajectorySetpoint / Offboard 位置或速度流在其它主模式下通常被忽略，会导致位置任务
        永远无法「到达」直至 deadline 超时；流式任务开始前必须处于 OFFBOARD。
        """
        self.spin_once()
        if self.current_nav_state() == NAVIGATION_STATE_OFFBOARD:
            return
        if not self._snapshot.connected:
            raise RuntimeError("飞控未连接，无法切入 OFFBOARD")
        self._prime_offboard_stream()
        task = TaskCommand(
            task_id="internal-stream-offboard",
            task_type=TaskType.MODE_SWITCH,
            payload={"mode": "OFFBOARD"},
        )
        self.publish_task(task)
        cmd = self.last_vehicle_command_sent()
        if cmd is None:
            raise RuntimeError("OFFBOARD 模式命令未发送")
        ack_ok, ack_detail = self.wait_vehicle_command_ack(
            cmd, timeout_ms=timeout_ack_ms
        )
        if not ack_ok:
            raise RuntimeError(f"切入 OFFBOARD ACK 失败: {ack_detail}")
        nav_ok, nav_detail = self.wait_nav_state(
            NAVIGATION_STATE_OFFBOARD, timeout_ms=timeout_nav_ms
        )
        if not nav_ok:
            raise RuntimeError(f"切入 OFFBOARD 后状态未就绪: {nav_detail}")

    def last_vehicle_command_sent(self) -> Optional[int]:
        """返回最近一次下发的 vehicle_command.command。"""
        return self._last_vehicle_command_sent

    def has_home_position(self) -> bool:
        """是否已获取到 home 点。"""
        return self._snapshot.home_valid

    def capture_home_at_current_position(self) -> bool:
        """解锁时将当前本地位置设为 home 点。"""
        snap = self._snapshot
        if not (snap.local_position_xy_valid and snap.local_position_z_valid):
            return False
        now_ms = int(time.time() * 1000)
        snap.home_position = (
            float(snap.position[0]),
            float(snap.position[1]),
            float(snap.position[2]),
        )
        snap.home_valid = True
        snap.home_updated_at_ms = now_ms
        snap.updated_at_ms = now_ms
        self._home_from_arm = True
        return True

    def clear_home_on_disarm(self) -> None:
        """上锁后清除 home，下次解锁重新记录。"""
        self._snapshot.home_valid = False
        self._snapshot.home_position = (0.0, 0.0, 0.0)
        self._snapshot.home_updated_at_ms = 0
        self._home_from_arm = False

    def wait_for_home_position(self, timeout_ms: int = 2000) -> bool:
        """等待 home 点就绪（飞行中重启时依赖 PX4 话题回退）。"""
        start = int(time.time() * 1000)
        while int(time.time() * 1000) - start <= timeout_ms:
            if self._snapshot.home_valid:
                return True
            self.spin_once()
            time.sleep(0.02)
        return self._snapshot.home_valid

    def wait_vehicle_command_ack(
        self, command: int, timeout_ms: int = 800
    ) -> tuple[bool, str]:
        """等待指定命令的 ACK。"""
        start = int(time.time() * 1000)
        while int(time.time() * 1000) - start <= timeout_ms:
            self.spin_once()
            ack = self._last_command_ack.get(command)
            if ack is not None:
                result, ts_ms = ack
                if ts_ms >= start:
                    if result == 0:
                        return True, "ACK_ACCEPTED"
                    return False, f"ACK_REJECTED(result={result})"
            time.sleep(0.01)
        return False, "ACK_TIMEOUT"

    def publish_task(self, task: TaskCommand) -> None:
        """发布控制任务到 PX4。"""
        if not self.is_ready:
            raise RuntimeError("ROS2 或 px4_msgs 不可用，无法发布任务")

        if task.task_type == TaskType.POSITION_CONTROL:
            self.publish_position_cmd(task)
        elif task.task_type == TaskType.VELOCITY_CONTROL:
            self.publish_velocity_cmd(task)
        elif task.task_type in {
            TaskType.MODE_SWITCH,
            TaskType.ARMING,
            TaskType.KILL_SWITCH,
        }:
            self._publish_vehicle_command(task)
        else:
            raise ValueError(f"不支持的任务类型: {task.task_type}")

    def publish_position_cmd(self, task: TaskCommand) -> None:
        """发布位置控制任务（外部 payload 为 ENU）。"""
        self._publish_position(
            float(task.payload.get("x", 0.0)),
            float(task.payload.get("y", 0.0)),
            float(task.payload.get("z", 0.0)),
            float(task.payload.get("yaw", 0.0)),
        )

    def publish_velocity_cmd(self, task: TaskCommand) -> None:
        """发布速度控制任务（外部 payload 为 ENU）。"""
        self._publish_velocity(
            float(task.payload.get("vx", 0.0)),
            float(task.payload.get("vy", 0.0)),
            float(task.payload.get("vz", 0.0)),
            float(task.payload.get("yaw", 0.0)),
        )

    def _publish_velocity(
        self, vx: float, vy: float, vz: float, yaw: float = 0.0
    ) -> None:
        """发布速度设定点（输入 ENU，发往 PX4 前转 NED）。"""
        if not self.is_ready:
            raise RuntimeError("ROS2 或 px4_msgs 不可用，无法发布实时速度指令")
        ned_vx, ned_vy, ned_vz = transform_world_vec((vx, vy, vz))
        self.publish_default_offboard(velocity=True)
        msg = TrajectorySetpoint()
        msg.timestamp = self._timestamp_us()
        msg.velocity = [float(ned_vx), float(ned_vy), float(ned_vz)]
        msg.yaw = float(convert_yaw(yaw))
        self.pub_trajectory.publish(msg)

    def _publish_position(self, x: float, y: float, z: float, yaw: float = 0.0) -> None:
        """发布位置设定点（输入 ENU，发往 PX4 前转 NED）。"""
        if not self.is_ready:
            raise RuntimeError("ROS2 或 px4_msgs 不可用，无法发布实时位置指令")
        ned_x, ned_y, ned_z = transform_world_vec((x, y, z))
        self.publish_default_offboard(position=True)
        msg = TrajectorySetpoint()
        msg.timestamp = self._timestamp_us()
        msg.position = [float(ned_x), float(ned_y), float(ned_z)]
        msg.yaw = float(convert_yaw(yaw))
        self.pub_trajectory.publish(msg)

    def _publish_composite(
        self,
        use_position: bool = False,
        use_velocity: bool = False,
        use_attitude: bool = False,
        use_acceleration: bool = False,
        use_jerk: bool = False,
        use_yaw_rate: bool = False,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        vx: float = 0.0,
        vy: float = 0.0,
        vz: float = 0.0,
        ax: float = 0.0,
        ay: float = 0.0,
        az: float = 0.0,
        jerk_x: float = 0.0,
        jerk_y: float = 0.0,
        jerk_z: float = 0.0,
        yaw_rate: float = 0.0,
        yaw: float = 0.0,
        qw: float = 1.0,
        qx: float = 0.0,
        qy: float = 0.0,
        qz: float = 0.0,
        thrust_z: float = 0.5,
    ) -> None:
        """发布复合设定点（输入 ENU/FLU，发往 PX4 前转 NED/FRD）。

        ``thrust_z`` 为 FLU 机体系 Z（向上为正；悬停约 +0.5）。
        """
        if not self.is_ready:
            raise RuntimeError("ROS2 或 px4_msgs 不可用，无法发布复合实时指令")

        self.publish_default_offboard(
            position=use_position,
            velocity=use_velocity,
            attitude=use_attitude,
            acceleration=use_acceleration,
        )

        if use_position or use_velocity or use_acceleration or use_jerk or use_yaw_rate:
            ts_msg = TrajectorySetpoint()
            ts_msg.timestamp = self._timestamp_us()
            if use_position:
                ned_x, ned_y, ned_z = transform_world_vec((x, y, z))
                ts_msg.position = [float(ned_x), float(ned_y), float(ned_z)]
            if use_velocity:
                ned_vx, ned_vy, ned_vz = transform_world_vec((vx, vy, vz))
                ts_msg.velocity = [float(ned_vx), float(ned_vy), float(ned_vz)]
            if use_acceleration:
                ned_ax, ned_ay, ned_az = transform_world_vec((ax, ay, az))
                ts_msg.acceleration = [float(ned_ax), float(ned_ay), float(ned_az)]
            if use_jerk:
                ned_jerk_x, ned_jerk_y, ned_jerk_z = transform_world_vec((jerk_x, jerk_y, jerk_z))
                ts_msg.jerk = [float(ned_jerk_x), float(ned_jerk_y), float(ned_jerk_z)]
            if use_yaw_rate:
                # ENU 上向角速度到 NED：竖直轴反向，只取反，不做偏航角的 π/2 平移。
                ts_msg.yawspeed = float(-yaw_rate)
            ts_msg.yaw = float(convert_yaw(yaw))
            self.pub_trajectory.publish(ts_msg)

        if use_attitude:
            _pos, q_ned = enu_to_ned((0.0, 0.0, 0.0), (qw, qx, qy, qz))
            _tx, _ty, thrust_frd_z = transform_body_vec((0.0, 0.0, thrust_z))
            att_msg = VehicleAttitudeSetpoint()
            att_msg.timestamp = self._timestamp_us()
            att_msg.q_d = [
                float(q_ned[0]),
                float(q_ned[1]),
                float(q_ned[2]),
                float(q_ned[3]),
            ]
            att_msg.thrust_body = [0.0, 0.0, float(thrust_frd_z)]
            self.pub_attitude_sp.publish(att_msg)

    def _prime_offboard_stream(
        self,
        duration_s: float = 0.6,
        rate_hz: float = 20.0,
    ) -> None:
        """在发送切入 OFFBOARD 的 vehicle_command 之前，短时发布位置 offboard 流。

        PX4 要求 offboard 控制链路已有效（否则常见 ``ACK_REJECTED(result=1)``）。
        使用当前快照位置（ENU）作为设定点，由 ``_publish_position`` 转 NED 下发。
        """
        if not self.is_ready:
            return
        px, py, pz = self._snapshot.position
        _roll, _pitch, yaw = euler_from_quat(
            self._snapshot.attitude_quat[0],
            self._snapshot.attitude_quat[1],
            self._snapshot.attitude_quat[2],
            self._snapshot.attitude_quat[3],
        )
        dt = 1.0 / max(rate_hz, 1.0)
        deadline = time.time() + duration_s
        while time.time() < deadline:
            self._publish_position(float(px), float(py), float(pz), yaw=float(yaw))
            self.spin_once()
            time.sleep(dt)

    def _publish_vehicle_command(self, task: TaskCommand) -> None:
        """发布模式切换/解锁上锁命令。"""
        payload = dict(task.payload)
        if task.task_type == TaskType.ARMING:
            payload = self._map_arming_payload(payload)
        elif task.task_type == TaskType.MODE_SWITCH:
            mode_raw = str(payload.get("mode", "")).upper().strip()
            if mode_raw == "OFFBOARD":
                self._prime_offboard_stream()
            payload = self._map_mode_switch_payload(payload)
        elif task.task_type == TaskType.KILL_SWITCH:
            self.publish_kill_switch()
            return

        msg = VehicleCommand()
        msg.timestamp = self._timestamp_us()
        msg.command = int(payload.get("command", 0))
        msg.param1 = float(payload.get("param1", 0.0))
        msg.param2 = float(payload.get("param2", 0.0))
        msg.param3 = float(payload.get("param3", 0.0))
        msg.param4 = float(payload.get("param4", 0.0))
        msg.param5 = float(payload.get("param5", 0.0))
        msg.param6 = float(payload.get("param6", 0.0))
        msg.param7 = float(payload.get("param7", 0.0))
        msg.target_system = int(payload.get("target_system", 1))
        msg.target_component = int(payload.get("target_component", 1))
        msg.source_system = int(payload.get("source_system", 1))
        msg.source_component = int(payload.get("source_component", 1))
        msg.from_external = True
        self.pub_vehicle_cmd.publish(msg)
        self._last_vehicle_command_sent = msg.command

    def publish_kill_switch(self) -> None:
        """发布 kill 开关命令。"""
        msg = VehicleCommand()
        msg.timestamp = self._timestamp_us()
        msg.command = 400
        msg.param1 = 0.0
        msg.param2 = 21196.0
        msg.param3 = 0.0
        msg.param4 = 0.0
        msg.param5 = 0.0
        msg.param6 = 0.0
        msg.param7 = 0.0
        msg.target_system = 1
        msg.target_component = 0
        msg.source_system = 0
        msg.source_component = 0
        msg.from_external = True
        self.pub_vehicle_cmd.publish(msg)

    def execute_return_home_and_land(
        self,
        timeout_ms: int = 20000,
        pos_tolerance_m: float = 0.5,
        phase_cb: Optional[Callable[[str, str], None]] = None,
    ) -> None:
        """自主返航：回到 home 点上方，然后发送 LAND。"""
        if not self._snapshot.home_valid:
            self.wait_for_home_position(timeout_ms=2000)
        if not self._snapshot.home_valid:
            raise RuntimeError("未获取到 home_position，无法执行返航")

        start = int(time.time() * 1000)
        home_x, home_y, _home_z = self._snapshot.home_position
        current_z = float(self._snapshot.position[2])
        _roll, _pitch, yaw = euler_from_quat(
            self._snapshot.attitude_quat[0],
            self._snapshot.attitude_quat[1],
            self._snapshot.attitude_quat[2],
            self._snapshot.attitude_quat[3],
        )
        if phase_cb is not None:
            phase_cb("RTH_NAVIGATING_HOME", "正在飞向 home 点上方")

        while int(time.time() * 1000) - start <= timeout_ms:
            self.spin_once()
            px, py, _pz = self._snapshot.position
            dist_xy = math.hypot(home_x - px, home_y - py)
            self._publish_position(home_x, home_y, current_z, yaw=float(yaw))
            if dist_xy <= pos_tolerance_m:
                break
            time.sleep(0.05)
        else:
            if phase_cb is not None:
                phase_cb("RTH_FAILED", "返航超时，未到达 home 点上方")
            raise RuntimeError("返航超时，未到达 home 点上方")

        if phase_cb is not None:
            phase_cb("RTH_LANDING", "已到达 home 点上方，开始降落")
        task = TaskCommand(
            task_id="internal-return-land",
            task_type=TaskType.MODE_SWITCH,
            payload={"mode": "LAND"},
        )
        self._publish_vehicle_command(task)
        command = self.last_vehicle_command_sent()
        if command is None:
            raise RuntimeError("LAND 命令发送失败")
        ack_ok, ack_detail = self.wait_vehicle_command_ack(command, timeout_ms=2000)
        if not ack_ok:
            if phase_cb is not None:
                phase_cb("RTH_FAILED", f"LAND ACK 失败: {ack_detail}")
            raise RuntimeError(f"LAND ACK 失败: {ack_detail}")
        if phase_cb is not None:
            phase_cb("RTH_DONE", "返航降落流程完成")

    def _map_arming_payload(self, payload: dict) -> dict:
        """将 ARMING 任务映射为 PX4 标准命令。"""
        command_component_arm_disarm = 400
        arm = bool(payload.get("arm", False))
        return {
            "command": command_component_arm_disarm,
            "param1": 1.0 if arm else 0.0,
            "target_system": int(payload.get("target_system", 1)),
            "target_component": int(payload.get("target_component", 1)),
            "source_system": int(payload.get("source_system", 1)),
            "source_component": int(payload.get("source_component", 1)),
        }

    def _map_mode_switch_payload(self, payload: dict) -> dict:
        """将模式字符串映射为 PX4 VehicleCommand。

        主模式切换使用 MAV_CMD_DO_SET_MODE(176)：param1=1 表示使用 PX4 custom main mode，
        param2 为 PX4_CUSTOM_MAIN_MODE_* 枚举值（与 PX4 `px4_custom_mode.h` 一致）。

        定点（位置模式）-> POSCTL(3)；自稳 -> STABILIZED(7)。
        LAND 使用 MAV_CMD_NAV_LAND(21)，不再误用主模式 5（ACRO）。
        """
        command_do_set_mode = 176
        command_nav_land = 21
        # PX4_CUSTOM_MAIN_MODE（与固件 px4_custom_mode.h 对齐）
        main_manual = 1
        main_altctl = 2
        main_posctl = 3
        main_auto = 4
        main_acro = 5
        main_offboard = 6
        main_stabilized = 7
        main_rattitude = 8

        mode = str(payload.get("mode", "")).upper().strip()
        # 别名：定点≈POSCTL，自稳≈STABILIZED；HOLD 切入 POSCTL（GPS 位置保持，非 AUTO 盘旋子模式）
        aliases = {
            "POSITION": "POSCTL",
            "定点": "POSCTL",
            "STAB": "STABILIZED",
            "自稳": "STABILIZED",
        }
        mode = aliases.get(mode, mode)

        if mode == "LAND":
            return {
                "command": command_nav_land,
                "param1": 0.0,
                "param2": 0.0,
                "param3": 0.0,
                "param4": 0.0,
                "param5": 0.0,
                "param6": 0.0,
                "param7": 0.0,
                "target_system": int(payload.get("target_system", 1)),
                "target_component": int(payload.get("target_component", 1)),
                "source_system": int(payload.get("source_system", 1)),
                "source_component": int(payload.get("source_component", 1)),
            }

        mode_map = {
            "MANUAL": main_manual,
            "ALTCTL": main_altctl,
            "ALTITUDE": main_altctl,
            "POSCTL": main_posctl,
            "AUTO": main_auto,
            "ACRO": main_acro,
            "OFFBOARD": main_offboard,
            "STABILIZED": main_stabilized,
            "RATTITUDE": main_rattitude,
            "HOLD": main_posctl,
        }
        if mode not in mode_map:
            raise ValueError(
                f"不支持的模式: {mode}；支持 MANUAL/ALTCTL/POSCTL/AUTO/ACRO/OFFBOARD/"
                f"STABILIZED/RATTITUDE/HOLD/LAND/RETURN_HOME(由节点单独处理)"
            )
        return {
            "command": command_do_set_mode,
            "param1": 1.0,
            "param2": float(mode_map[mode]),
            "param3": 0.0,
            "param4": 0.0,
            "param5": 0.0,
            "param6": 0.0,
            "param7": 0.0,
            "target_system": int(payload.get("target_system", 1)),
            "target_component": int(payload.get("target_component", 1)),
            "source_system": int(payload.get("source_system", 1)),
            "source_component": int(payload.get("source_component", 1)),
        }

    @staticmethod
    def _timestamp_us() -> int:
        """生成 PX4 使用的微秒时间戳。"""
        return int(time.time() * 1_000_000)

    @staticmethod
    def _optional_float(value: Any) -> Optional[float]:
        """把可选数值统一转换为 float，None 保持为空。"""
        if value is None:
            return None
        return float(value)


    def clear_hover(self) -> None:
        """清除悬停锁存，下次 publish_hover 将重新捕获当前位置。"""
        self._hover_setpoint = None

    def publish_hover(self, *, recapture: bool = False) -> None:
        """位置控制悬停在当前位置。

        首次调用（或 ``recapture=True`` / ``clear_hover`` 之后）锁存当前 ENU 位姿与偏航，
        之后重复调用继续发布同一设定点，避免跟随估计噪声漂移。
        """
        if not self.is_ready:
            raise RuntimeError("ROS2 或 px4_msgs 不可用，无法发布悬停指令")
        if recapture or self._hover_setpoint is None:
            px, py, pz = self._snapshot.position
            _roll, _pitch, yaw = euler_from_quat(
                self._snapshot.attitude_quat[0],
                self._snapshot.attitude_quat[1],
                self._snapshot.attitude_quat[2],
                self._snapshot.attitude_quat[3],
            )
            self._hover_setpoint = (float(px), float(py), float(pz), float(yaw))
        x, y, z, yaw = self._hover_setpoint
        self._publish_composite(
            use_position=True,
            use_velocity=False,
            use_attitude=False,
            x=x,
            y=y,
            z=z,
            yaw=yaw,
        )

    def publish_composite_setpoint(
        self,
        *,
        use_position: bool = False,
        use_velocity: bool = False,
        use_attitude: bool = False,
        use_acceleration: bool = False,
        use_jerk: bool = False,
        use_yaw_rate: bool = False,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        vx: float = 0.0,
        vy: float = 0.0,
        vz: float = 0.0,
        ax: float = 0.0,
        ay: float = 0.0,
        az: float = 0.0,
        jerk_x: float = 0.0,
        jerk_y: float = 0.0,
        jerk_z: float = 0.0,
        yaw_rate: float = 0.0,
        yaw: float = 0.0,
        qw: float = 1.0,
        qx: float = 0.0,
        qy: float = 0.0,
        qz: float = 0.0,
        thrust_z: float = 0.5,
    ) -> None:
        """发布复合实时控制设定点（外部 ENU/FLU，内部由 ``_publish_composite`` 转 NED/FRD）。"""
        self._publish_composite(
            use_position=use_position,
            use_velocity=use_velocity,
            use_attitude=use_attitude,
            use_acceleration=use_acceleration,
            use_jerk=use_jerk,
            use_yaw_rate=use_yaw_rate,
            x=x,
            y=y,
            z=z,
            vx=vx,
            vy=vy,
            vz=vz,
            ax=ax,
            ay=ay,
            az=az,
            jerk_x=jerk_x,
            jerk_y=jerk_y,
            jerk_z=jerk_z,
            yaw_rate=yaw_rate,
            yaw=yaw,
            qw=qw,
            qx=qx,
            qy=qy,
            qz=qz,
            thrust_z=thrust_z,
        )

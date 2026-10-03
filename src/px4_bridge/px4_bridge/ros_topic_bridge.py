"""ROS 话题桥接适配层（自定义消息）。"""

from __future__ import annotations

from typing import Any, Callable, Dict, Optional, TYPE_CHECKING
import logging
import time

from .task_server import TaskServerAdapter

from .realtime_controller import RealtimeController

try:
    import rclpy
    from rclpy.node import Node
    from px4_bridge_msgs.msg import BridgeStatus, RealtimeControl, TaskCommand

    ROS_TOPIC_BRIDGE_AVAILABLE = True
except ImportError:
    ROS_TOPIC_BRIDGE_AVAILABLE = False
    rclpy = None  # type: ignore[assignment]
    Node = Any  # type: ignore[misc,assignment]
    BridgeStatus = Any  # type: ignore[misc,assignment]
    RealtimeControl = Any  # type: ignore[misc,assignment]
    TaskCommand = Any  # type: ignore[misc,assignment]


def payload_from_task_command_msg(msg: Any) -> Dict[str, Any]:
    """从扁平 TaskCommand 字段按 task_type 组装内部 payload dict。"""
    task_type = str(getattr(msg, "task_type", "")).strip().upper()
    payload: Dict[str, Any] = {}

    if task_type == "ARMING":
        payload["arm"] = bool(getattr(msg, "arm", False))
        if bool(getattr(msg, "confirm_current_mode", False)):
            payload["confirm_current_mode"] = True
    elif task_type == "MODE_SWITCH":
        payload["mode"] = str(getattr(msg, "mode", "") or "")
    elif task_type == "POSITION_CONTROL":
        payload["x"] = float(getattr(msg, "x", 0.0))
        payload["y"] = float(getattr(msg, "y", 0.0))
        payload["z"] = float(getattr(msg, "z", 0.0))
        payload["yaw"] = float(getattr(msg, "yaw", 0.0))
        for key in ("arrival_radius_m", "arrival_dz_m", "arrival_speed_mps"):
            val = float(getattr(msg, key, 0.0) or 0.0)
            if val != 0.0:
                payload[key] = val
    elif task_type == "VELOCITY_CONTROL":
        payload["vx"] = float(getattr(msg, "vx", 0.0))
        payload["vy"] = float(getattr(msg, "vy", 0.0))
        payload["vz"] = float(getattr(msg, "vz", 0.0))
        payload["yaw"] = float(getattr(msg, "yaw", 0.0))
    elif task_type == "KILL_SWITCH":
        pass
    else:
        # 未知类型：尽量带上常见字段，交给 task_server 校验失败
        payload["arm"] = bool(getattr(msg, "arm", False))
        mode = str(getattr(msg, "mode", "") or "")
        if mode:
            payload["mode"] = mode
        payload["x"] = float(getattr(msg, "x", 0.0))
        payload["y"] = float(getattr(msg, "y", 0.0))
        payload["z"] = float(getattr(msg, "z", 0.0))

    return payload


class RosTopicBridgeAdapter:
    """对外 ROS2 自定义话题：task_cmd / realtime_control / status。"""

    def __init__(
        self,
        task_adapter: TaskServerAdapter,
        realtime_controller: "RealtimeController",
        status_provider: Callable[[], Dict[str, Any]],
        *,
        node: Optional[Node] = None,
    ) -> None:
        self.logger = logging.getLogger(self.__class__.__name__)
        self._task_adapter = task_adapter
        self._realtime_controller = realtime_controller
        self._status_provider = status_provider
        self._node = node
        self._pub_status = None

        if not ROS_TOPIC_BRIDGE_AVAILABLE:
            self.logger.warning("ROS 自定义消息不可用，跳过 ROS 话题桥接")
            return
        if self._node is None:
            return

        qos = 10
        self._node.create_subscription(
            TaskCommand,
            "/px4_bridge/in/task_cmd",
            self._on_task_cmd,
            qos,
        )
        self._node.create_subscription(
            RealtimeControl,
            "/px4_bridge/in/realtime_control",
            self._on_realtime_control,
            qos,
        )
        self._pub_status = self._node.create_publisher(
            BridgeStatus,
            "/px4_bridge/out/status",
            qos,
        )

    def spin_once(self) -> None:
        if self._node is not None and rclpy is not None:
            try:
                rclpy.spin_once(self._node, timeout_sec=0.0)
            except IndexError:
                pass

    def publish_status(self) -> None:
        if self._pub_status is None:
            return
        status = self._status_provider()
        drone = status.get("drone", {})
        realtime = status.get("realtime", {})
        msg = BridgeStatus()
        msg.lifecycle_state = str(status.get("lifecycle_state", "UNKNOWN"))
        msg.active_task_id = str(status.get("active_task_id") or "")
        msg.error = str(status.get("error") or "")
        msg.rth_phase = str(status.get("rth_phase", "IDLE"))
        msg.rth_detail = str(status.get("rth_detail", ""))
        msg.connected = bool(drone.get("connected", False))
        msg.mode = str(drone.get("mode", "UNKNOWN"))
        msg.armed = bool(drone.get("armed", False))
        msg.battery_remaining = float(drone.get("battery_remaining", 0.0))
        msg.failsafe = bool(drone.get("failsafe", False))
        msg.failsafe_reason = str(drone.get("failsafe_reason") or "")
        msg.position = [float(v) for v in drone.get("position", [0.0, 0.0, 0.0])]
        msg.velocity = [float(v) for v in drone.get("velocity", [0.0, 0.0, 0.0])]
        msg.attitude_quat = [
            float(v) for v in drone.get("attitude_quat", [1.0, 0.0, 0.0, 0.0])
        ]
        msg.updated_at_ms = int(drone.get("updated_at_ms", 0))
        msg.realtime_enabled = bool(realtime.get("enabled", False))
        msg.realtime_last_action = str(realtime.get("last_action", "IDLE"))
        self._pub_status.publish(msg)

    def close(self) -> None:
        """解除绑定（不销毁共享 Node，由 PX4BridgeNode 负责）。"""
        self._node = None
        self._pub_status = None

    def _on_task_cmd(self, msg: Any) -> None:
        self.logger.info("on_task_cmd %s", msg)
        try:
            payload = payload_from_task_command_msg(msg)
            raw_task: Dict[str, Any] = {
                "task_id": str(getattr(msg, "task_id", "")),
                "task_type": str(getattr(msg, "task_type", "")),
                "payload": payload,
            }
            deadline_ms = int(getattr(msg, "deadline_ms", 0))
            if deadline_ms > 0:
                raw_task["deadline_ms"] = deadline_ms
            self._task_adapter.submit(raw_task)
        except Exception as exc:  # pylint: disable=broad-except
            self.logger.error("task_cmd 提交失败: %s", exc)

    def _on_realtime_control(self, msg: Any) -> None:
        """接收实时控制：按 enable / has_command / use_* 使能并缓存指令。"""
        enable = bool(getattr(msg, "enable", False))
        has_command = bool(getattr(msg, "has_command", False))
        if enable or has_command:
            self._realtime_controller.enable()
        else:
            self._realtime_controller.disable()
            return

        if not has_command:
            return

        ts_ms = int(time.time() * 1000)

        try:
            self._realtime_controller.submit_from_dict(
                {
                    "x": float(getattr(msg, "x", 0.0)),
                    "y": float(getattr(msg, "y", 0.0)),
                    "z": float(getattr(msg, "z", 0.0)),
                    "use_position": bool(getattr(msg, "use_position", False)),
                    "use_velocity": bool(getattr(msg, "use_velocity", False)),
                    "use_attitude": bool(getattr(msg, "use_attitude", False)),
                    "use_acceleration": bool(getattr(msg, "use_acceleration", False)),
                    "use_jerk": bool(getattr(msg, "use_jerk", False)),
                    "use_yaw_rate": bool(getattr(msg, "use_yaw_rate", False)),
                    "vx": float(getattr(msg, "vx", 0.0)),
                    "vy": float(getattr(msg, "vy", 0.0)),
                    "vz": float(getattr(msg, "vz", 0.0)),
                    "ax": float(getattr(msg, "ax", 0.0)),
                    "ay": float(getattr(msg, "ay", 0.0)),
                    "az": float(getattr(msg, "az", 0.0)),
                    "jerk_x": float(getattr(msg, "jerk_x", 0.0)),
                    "jerk_y": float(getattr(msg, "jerk_y", 0.0)),
                    "jerk_z": float(getattr(msg, "jerk_z", 0.0)),
                    "yaw_rate": float(getattr(msg, "yaw_rate", 0.0)),
                    "yaw": float(getattr(msg, "yaw", 0.0)),
                    "qw": float(getattr(msg, "qw", 1.0)),
                    "qx": float(getattr(msg, "qx", 0.0)),
                    "qy": float(getattr(msg, "qy", 0.0)),
                    "qz": float(getattr(msg, "qz", 0.0)),
                    "thrust_z": float(getattr(msg, "thrust_z", -0.5)),
                    "timestamp_ms": ts_ms,
                }
            )
        except Exception as exc:  # pylint: disable=broad-except
            self.logger.error("实时控制指令无效: %s", exc)

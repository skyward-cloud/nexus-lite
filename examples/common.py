"""共用工具：发布 TaskCommand / RealtimeControl，订阅 BridgeStatus。"""

from __future__ import annotations

import math
import time
from typing import Any, Optional

import rclpy
from px4_bridge_msgs.msg import BridgeStatus, RealtimeControl, TaskCommand
from rclpy.node import Node
from rclpy.qos import QoSProfile


def apply_payload_to_task_command(msg: TaskCommand, payload: dict[str, Any]) -> None:
    """将内部 payload dict 写入扁平 TaskCommand 字段。"""
    if "arm" in payload:
        msg.arm = bool(payload["arm"])
    if "confirm_current_mode" in payload:
        msg.confirm_current_mode = bool(payload["confirm_current_mode"])
    if "mode" in payload:
        msg.mode = str(payload["mode"])
    for key in ("x", "y", "z", "vx", "vy", "vz", "yaw"):
        if key in payload:
            setattr(msg, key, float(payload[key]))
    for key in ("arrival_radius_m", "arrival_dz_m", "arrival_speed_mps"):
        if key in payload:
            setattr(msg, key, float(payload[key]))


class BridgeClient(Node):
    """轻量示例客户端。"""

    def __init__(self, node_name: str = "px4_bridge_example") -> None:
        super().__init__(node_name)
        qos = QoSProfile(depth=10)
        self._pub_task = self.create_publisher(
            TaskCommand, "/px4_bridge/in/task_cmd", qos
        )
        self._pub_rt = self.create_publisher(
            RealtimeControl, "/px4_bridge/in/realtime_control", qos
        )
        self._status: Optional[BridgeStatus] = None
        self.create_subscription(
            BridgeStatus, "/px4_bridge/out/status", self._on_status, qos
        )

    def _on_status(self, msg: BridgeStatus) -> None:
        self._status = msg

    def spin_brief(self, duration_s: float = 0.05) -> None:
        end = time.time() + duration_s
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def latest_status(self) -> Optional[BridgeStatus]:
        """返回最近一次 /px4_bridge/out/status，不额外阻塞。"""
        return self._status

    def wait_status(self, timeout_s: float = 5.0) -> BridgeStatus:
        end = time.time() + timeout_s
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._status is not None:
                return self._status
        raise TimeoutError("未收到 /px4_bridge/out/status，请确认 bridge 已启动")

    def status(self) -> BridgeStatus:
        self.spin_brief(0.1)
        if self._status is None:
            return self.wait_status()
        return self._status

    def submit_task(
        self,
        task_id: str,
        task_type: str,
        payload: dict[str, Any],
        *,
        deadline_ms: int = 0,
    ) -> None:
        msg = TaskCommand()
        msg.task_id = task_id
        msg.task_type = task_type
        msg.deadline_ms = int(deadline_ms)
        apply_payload_to_task_command(msg, payload)
        self._pub_task.publish(msg)
        self.get_logger().info(f"submit {task_id} {task_type} {payload}")

    def publish_realtime(
        self,
        *,
        enable: bool = True,
        has_command: bool = True,
        vx: float = 0.0,
        vy: float = 0.0,
        vz: float = 0.0,
        yaw: float = 0.0,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        use_position: bool = False,
        use_velocity: bool = False,
        use_attitude: bool = False,
        qw: float = 1.0,
        qx: float = 0.0,
        qy: float = 0.0,
        qz: float = 0.0,
        thrust_z: float = 0.5,
    ) -> None:
        msg = RealtimeControl()
        msg.enable = enable
        msg.has_command = has_command
        msg.mode = 0  # 已废弃，控制类型由 use_* 决定
        msg.use_position = use_position
        msg.use_velocity = use_velocity
        msg.use_attitude = use_attitude
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        msg.vx = float(vx)
        msg.vy = float(vy)
        msg.vz = float(vz)
        msg.yaw = float(yaw)
        msg.qw = float(qw)
        msg.qx = float(qx)
        msg.qy = float(qy)
        msg.qz = float(qz)
        msg.thrust_z = float(thrust_z)
        msg.timestamp_ms = int(time.time() * 1000)
        self._pub_rt.publish(msg)

    def wait_lifecycle(
        self,
        expected: str,
        timeout_s: float = 30.0,
        *,
        idle_when_ready: bool = False,
    ) -> BridgeStatus:
        """等待 lifecycle_state；若 idle_when_ready 且 READY 且无活跃任务则返回。"""
        end = time.time() + timeout_s
        while time.time() < end:
            st = self.status()
            life = st.lifecycle_state
            active = (st.active_task_id or "").strip()
            if life == expected:
                if not idle_when_ready or not active:
                    return st
            time.sleep(0.1)
        raise TimeoutError(f"等待 lifecycle={expected} 超时")

    def wait_task_idle(
        self,
        timeout_s: float = 120.0,
        *,
        after_task_id: Optional[str] = None,
        start_grace_s: float = 0.8,
    ) -> BridgeStatus:
        """等待回到 READY 且无 active_task。

        若指定 after_task_id：优先等到该 id 变为 active 再等结束。
        ARMING 等短任务可能在两次轮询间完成，故在 start_grace_s 后若已是
        READY 且空闲，视为已完成（避免永远等不到 active）。
        实时控制占着 EXECUTING 时不会误判为空闲。
        """
        end = time.time() + timeout_s
        t0 = time.time()
        saw_active = after_task_id is None
        while time.time() < end:
            st = self.status()
            if st.lifecycle_state == "FAULT":
                raise RuntimeError(f"bridge FAULT: {st.error}")
            active = (st.active_task_id or "").strip()
            ready_idle = st.lifecycle_state == "READY" and not active
            if not saw_active:
                if active == after_task_id:
                    saw_active = True
                elif ready_idle and (time.time() - t0) >= start_grace_s:
                    return st
                time.sleep(0.05)
                continue
            if ready_idle:
                return st
            time.sleep(0.1)
        raise TimeoutError(
            f"等待任务空闲超时"
            + (f" (after_task_id={after_task_id})" if after_task_id else "")
        )

    def wait_armed(self, armed: bool = True, timeout_s: float = 15.0) -> BridgeStatus:
        """等待 BridgeStatus.armed 达到期望值。"""
        end = time.time() + timeout_s
        while time.time() < end:
            st = self.status()
            if st.lifecycle_state == "FAULT":
                raise RuntimeError(f"bridge FAULT: {st.error}")
            if bool(st.armed) == bool(armed):
                return st
            time.sleep(0.1)
        raise TimeoutError(f"等待 armed={armed} 超时")

    def wait_until_arrived(
        self,
        x: float,
        y: float,
        z: float,
        *,
        radius_m: float = 0.3,
        dz_m: float = 0.2,
        speed_mps: float = 0.5,
        timeout_s: float = 90.0,
    ) -> BridgeStatus:
        """轮询 BridgeStatus，直到当前位置进入到点半径（水平/高度/速度）。"""
        end = time.time() + timeout_s
        last_horiz = last_dz = last_speed = float("nan")
        while time.time() < end:
            st = self.status()
            if st.lifecycle_state == "FAULT":
                raise RuntimeError(f"bridge FAULT: {st.error}")
            px, py, pz = (float(st.position[0]), float(st.position[1]), float(st.position[2]))
            vx, vy, vz = (float(st.velocity[0]), float(st.velocity[1]), float(st.velocity[2]))
            last_horiz = math.hypot(x - px, y - py)
            last_dz = abs(z - pz)
            last_speed = math.sqrt(vx * vx + vy * vy + vz * vz)
            if last_horiz <= radius_m and last_dz <= dz_m and last_speed <= speed_mps:
                return st
            time.sleep(0.1)
        raise TimeoutError(
            f"到点超时 target=({x:.2f},{y:.2f},{z:.2f}) "
            f"Δxy={last_horiz:.2f}m Δz={last_dz:.2f}m |v|={last_speed:.2f}m/s"
        )


def make_client(node_name: str = "px4_bridge_example") -> BridgeClient:
    if not rclpy.ok():
        rclpy.init(args=[])
    return BridgeClient(node_name)

"""ROS2 桥：订阅状态/点云，发布 task_cmd。"""

from __future__ import annotations

import threading
import time
from typing import Any, Callable, Optional

import numpy as np
import rclpy
from px4_bridge_msgs.msg import BridgeStatus, TaskCommand
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2


StatusCallback = Callable[[dict[str, Any]], None]
CloudCallback = Callable[[list[float], int], None]


def _apply_payload_to_task_command(msg: TaskCommand, payload: dict[str, Any]) -> None:
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


def _pointcloud2_xyz(msg: PointCloud2, max_points: int = 80000) -> np.ndarray:
    """从 PointCloud2 提取 xyz，返回 (N, 3) float32。"""
    fields = {f.name: f for f in msg.fields}
    if not all(k in fields for k in ("x", "y", "z")):
        return np.zeros((0, 3), dtype=np.float32)

    point_step = int(msg.point_step)
    n = int(msg.width) * int(msg.height)
    if n <= 0 or point_step <= 0:
        return np.zeros((0, 3), dtype=np.float32)

    raw = np.frombuffer(msg.data, dtype=np.uint8)
    if raw.size < n * point_step:
        n = raw.size // point_step
    if n <= 0:
        return np.zeros((0, 3), dtype=np.float32)

    step = max(1, n // max_points)
    count = (n + step - 1) // step
    view = np.lib.stride_tricks.as_strided(
        raw,
        shape=(n, point_step),
        strides=(point_step, 1),
        writeable=False,
    )[::step][:count]

    ox = int(fields["x"].offset)
    oy = int(fields["y"].offset)
    oz = int(fields["z"].offset)
    x = view[:, ox : ox + 4].copy().view(np.float32).reshape(-1)
    y = view[:, oy : oy + 4].copy().view(np.float32).reshape(-1)
    z = view[:, oz : oz + 4].copy().view(np.float32).reshape(-1)
    pts = np.column_stack((x, y, z))
    finite = np.isfinite(pts).all(axis=1)
    return pts[finite]


class WebViewRosNode(Node):
    """web_view 专用 ROS 节点。"""

    def __init__(
        self,
        *,
        status_cb: StatusCallback,
        cloud_cb: CloudCallback,
        cloud_topic: str = "/lidar/points_world",
        max_cloud_points: int = 20000,
    ) -> None:
        super().__init__("px4_bridge_web_view")
        self._status_cb = status_cb
        self._cloud_cb = cloud_cb
        self._max_cloud_points = max_cloud_points
        self._task_seq = 0
        self._last_cloud_push_ms = 0
        self._cloud_min_interval_ms = 100  # 最多 ~10Hz 推前端

        qos = QoSProfile(depth=10)
        self._pub_task = self.create_publisher(
            TaskCommand, "/px4_bridge/in/task_cmd", qos
        )
        self.create_subscription(
            BridgeStatus, "/px4_bridge/out/status", self._on_status, qos
        )

        # 点云多为 Best Effort
        cloud_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            PointCloud2, cloud_topic, self._on_cloud, cloud_qos
        )
        self.get_logger().info(f"订阅点云: {cloud_topic}")

    def _on_status(self, msg: BridgeStatus) -> None:
        payload = {
            "lifecycle_state": str(msg.lifecycle_state),
            "active_task_id": str(msg.active_task_id or ""),
            "error": str(msg.error or ""),
            "rth_phase": str(msg.rth_phase or ""),
            "rth_detail": str(msg.rth_detail or ""),
            "connected": bool(msg.connected),
            "mode": str(msg.mode),
            "armed": bool(msg.armed),
            "battery_remaining": float(msg.battery_remaining),
            "failsafe": bool(msg.failsafe),
            "failsafe_reason": str(msg.failsafe_reason or ""),
            "position": [float(msg.position[0]), float(msg.position[1]), float(msg.position[2])],
            "velocity": [float(msg.velocity[0]), float(msg.velocity[1]), float(msg.velocity[2])],
            "attitude_quat": [
                float(msg.attitude_quat[0]),
                float(msg.attitude_quat[1]),
                float(msg.attitude_quat[2]),
                float(msg.attitude_quat[3]),
            ],
            "updated_at_ms": int(msg.updated_at_ms),
            "realtime_enabled": bool(msg.realtime_enabled),
            "realtime_last_action": str(msg.realtime_last_action or ""),
        }
        try:
            self._status_cb(payload)
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().warning(f"status_cb 失败: {exc}")

    def _on_cloud(self, msg: PointCloud2) -> None:
        now_ms = int(time.time() * 1000)
        if now_ms - self._last_cloud_push_ms < self._cloud_min_interval_ms:
            return
        self._last_cloud_push_ms = now_ms
        try:
            xyz = _pointcloud2_xyz(msg, max_points=self._max_cloud_points)
            # 扁平数组给 Three.js BufferAttribute
            flat = xyz.reshape(-1).tolist()
            self._cloud_cb(flat, int(xyz.shape[0]))
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().warning(f"点云解析失败: {exc}")

    def submit_task(
        self,
        task_type: str,
        payload: dict[str, Any],
        *,
        task_id: Optional[str] = None,
        deadline_ms: int = 0,
    ) -> str:
        self._task_seq += 1
        tid = task_id or f"web-{task_type.lower()}-{self._task_seq}"
        msg = TaskCommand()
        msg.task_id = tid
        msg.task_type = str(task_type)
        msg.deadline_ms = int(deadline_ms)
        _apply_payload_to_task_command(msg, payload)
        self._pub_task.publish(msg)
        self.get_logger().info(f"task_cmd {tid} {task_type} {payload}")
        return tid


class RosBridge:
    """在后台线程运行 rclpy，供 pywebview 调用。"""

    def __init__(
        self,
        *,
        status_cb: StatusCallback,
        cloud_cb: CloudCallback,
        cloud_topic: str = "/lidar/points_world",
    ) -> None:
        self._status_cb = status_cb
        self._cloud_cb = cloud_cb
        self._cloud_topic = cloud_topic
        self._node: Optional[WebViewRosNode] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._lock = threading.Lock()

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="ros-bridge", daemon=True)
        self._thread.start()
        if not self._ready.wait(timeout=10.0):
            raise RuntimeError("ROS 节点启动超时")

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            self._thread = None

    def submit_task(
        self,
        task_type: str,
        payload: dict[str, Any],
        **kwargs: Any,
    ) -> str:
        with self._lock:
            if self._node is None:
                raise RuntimeError("ROS 节点未就绪")
            return self._node.submit_task(task_type, payload, **kwargs)

    def _run(self) -> None:
        if not rclpy.ok():
            rclpy.init(args=None)
        node = WebViewRosNode(
            status_cb=self._status_cb,
            cloud_cb=self._cloud_cb,
            cloud_topic=self._cloud_topic,
        )
        self._node = node
        self._ready.set()
        try:
            while not self._stop.is_set() and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.05)
        finally:
            with self._lock:
                self._node = None
            node.destroy_node()

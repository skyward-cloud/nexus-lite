"""精简主控制节点：连接监控、流式设定点、实时控制、离散任务。"""

from __future__ import annotations

import logging
import math
import time
from pathlib import Path
from typing import Any, Optional, Tuple

from .ros_topic_bridge import RosTopicBridgeAdapter
from .unification_adapter import PX4TopicsAdapter
from .lifecycle import Lifecycle
from .models import DroneSnapshot, LifecycleState, RuntimeContext, TaskCommand, TaskType
from .realtime_controller import RealtimeController
from .task_server import TaskServerAdapter

try:
    import rclpy
    from rclpy.node import Node as RosNode

    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    rclpy = None  # type: ignore[assignment]
    RosNode = Any  # type: ignore[misc,assignment]

ROS_NODE_NAME = "px4_bridge"


class PX4BridgeNode:
    """lite 控制环节点（进程内单一 ROS Node）。"""

    _ROS_SPIN_BATCH = 12
    _STREAM_TYPES = frozenset(
        {
            TaskType.POSITION_CONTROL,
            TaskType.VELOCITY_CONTROL,
        }
    )

    def __init__(
        self,
        *,
        loop_hz: int = 20,
        connection_timeout_ms: int = 1000,
        enable_file_logging: bool = False,
        log_file_path: str = "logs/px4_bridge.log",
        px4_adapter: Optional[PX4TopicsAdapter] = None,
        task_adapter: Optional[TaskServerAdapter] = None,
        realtime_controller: Optional[RealtimeController] = None,
        ros_topic_bridge: Optional[RosTopicBridgeAdapter] = None,
        ros_node: Optional[RosNode] = None,
        owns_rclpy: bool = False,
    ) -> None:
        self.logger = logging.getLogger(self.__class__.__name__)
        self.loop_hz = max(1, int(loop_hz))
        self.connection_timeout_ms = int(connection_timeout_ms)
        self._setup_file_logging(enable_file_logging, log_file_path)

        self.ctx = RuntimeContext()
        self.sm = Lifecycle(self.ctx)

        self._owns_rclpy = owns_rclpy
        self._ros_node = ros_node
        if self._ros_node is None:
            self._ros_node, self._owns_rclpy = self._create_ros_node()

        self.px4_adapter = px4_adapter or PX4TopicsAdapter(self._ros_node)
        self.task_adapter = task_adapter or TaskServerAdapter()
        self.realtime_controller = realtime_controller or RealtimeController(
            self.px4_adapter
        )
        self.ros_topic_bridge = ros_topic_bridge or RosTopicBridgeAdapter(
            task_adapter=self.task_adapter,
            realtime_controller=self.realtime_controller,
            status_provider=self.runtime_status,
            node=self._ros_node,
        )

        self._running = False
        self._last_armed_state: Optional[bool] = None
        self._history: list[dict] = []
        self.last_tr_reason = ""
        self.stream_pending_task: Optional[TaskCommand] = None
        self.stream_pending_start_ms: int = 0

    @staticmethod
    def _create_ros_node() -> Tuple[Optional[RosNode], bool]:
        """创建 ROS Node"""
        if not ROS2_AVAILABLE or rclpy is None:
            return None, False
        owns_rclpy = False
        if not rclpy.ok():
            # 允许 launch / ros2 run 的 __node:= 重映射生效（仅单 Node）。
            rclpy.init()
            owns_rclpy = True
        return RosNode(ROS_NODE_NAME), owns_rclpy

    @classmethod
    def create_default(
        cls,
        *,
        loop_hz: int = 20,
        connection_timeout_ms: int = 1000,
        enable_file_logging: bool = False,
        log_file_path: str = "logs/px4_bridge.log",
        realtime_timeout_ms: int = 300,
        realtime_max_speed_mps: float = 2.0,
    ) -> "PX4BridgeNode":
        ros_node, owns_rclpy = cls._create_ros_node()
        px4_adapter = PX4TopicsAdapter(ros_node)
        task_adapter = TaskServerAdapter()
        realtime = RealtimeController(
            px4_adapter,
            timeout_ms=realtime_timeout_ms,
            max_speed_mps=realtime_max_speed_mps,
        )
        return cls(
            loop_hz=loop_hz,
            connection_timeout_ms=connection_timeout_ms,
            enable_file_logging=enable_file_logging,
            log_file_path=log_file_path,
            px4_adapter=px4_adapter,
            task_adapter=task_adapter,
            realtime_controller=realtime,
            ros_node=ros_node,
            owns_rclpy=owns_rclpy,
        )

    def _setup_file_logging(self, enabled: bool, log_file_path: str) -> None:
        if not enabled:
            return
        path = Path(log_file_path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        abs_path = str(path.resolve())
        for handler in self.logger.handlers:
            if isinstance(handler, logging.FileHandler):
                try:
                    if Path(handler.baseFilename).resolve() == Path(abs_path):
                        return
                except OSError:
                    continue
        file_handler = logging.FileHandler(abs_path, encoding="utf-8")
        file_handler.setLevel(logging.INFO)
        file_handler.setFormatter(
            logging.Formatter("%(asctime)s %(levelname)s [%(name)s] %(message)s")
        )
        self.logger.addHandler(file_handler)

    def request_stop(self) -> None:
        """请求退出主循环（不释放资源）。"""
        self._running = False

    def start(self) -> None:
        self.logger.info("节点启动")
        self._running = True
        ok, from_s, to_s, reason = self.sm.transit(
            LifecycleState.WAITING_CONNECTION, "系统启动"
        )
        if ok:
            self._log_transition(from_s.value, to_s.value, reason)

        interval_s = 1.0 / self.loop_hz
        next_tick = time.monotonic()
        while self._running:
            now = time.monotonic()
            if now < next_tick:
                time.sleep(min(next_tick - now, interval_s))
                continue
            self._tick()
            next_tick += interval_s
            if next_tick <= now:
                next_tick = now + interval_s

    def stop(self) -> None:
        self._running = False
        if self.sm.state == LifecycleState.SHUTDOWN:
            return
        self.px4_adapter.close()
        self.ros_topic_bridge.close()
        self._shutdown_ros()
        ok, from_s, to_s, reason = self.sm.transit(LifecycleState.SHUTDOWN, "停止请求")
        if ok:
            self._log_transition(from_s.value, to_s.value, reason)
        self.logger.info("节点停止")

    def _spin_batch(self) -> None:
        if self._ros_node is None or rclpy is None:
            return
        for _ in range(self._ROS_SPIN_BATCH):
            try:
                rclpy.spin_once(self._ros_node, timeout_sec=0.0)
            except IndexError:
                # ROS2 Humble 在 QoS event 重建时偶发 wait set 越界。
                break

    def _shutdown_ros(self) -> None:
        if self._ros_node is not None:
            try:
                self._ros_node.destroy_node()
            except Exception as exc:  # pylint: disable=broad-except
                self.logger.debug("destroy_node 异常: %s", exc)
            self._ros_node = None
        if self._owns_rclpy and rclpy is not None and rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception as exc:  # pylint: disable=broad-except
                self.logger.debug("rclpy.shutdown 异常: %s", exc)
            self._owns_rclpy = False

    def _tick(self) -> None:
        self._spin_batch()
        snapshot = self.px4_adapter.get_snapshot()
        if self._handle_armed_state_transitions(snapshot):
            self.ros_topic_bridge.publish_status()
            return
        self._run_control_loop(snapshot)
        self.ros_topic_bridge.publish_status()

    def _run_control_loop(self, snapshot: DroneSnapshot) -> None:
        if self._process_connection(snapshot):
            return
        # 实时控制优先：使能后立即抢占流式任务并下发，避免被 POSITION 流挡住
        if self.realtime_controller.enabled():
            if self.realtime_controller.process_tick(
                self.sm,
                self.ctx,
                stream_abort=self._abort_stream,
                append_record=self._append_record,
                to_fault=self._to_fault,
                log_transition=self._log_transition,
            ):
                return
        # 有更新任务时立即抢占当前流式任务
        if self.stream_pending_task is not None and self.task_adapter.pending_count() > 0:
            self._abort_stream("新任务抢占，中止当前流式任务", status="CANCELLED")
        if self.stream_pending_task is not None:
            if self._process_stream(snapshot):
                return
        self._process_discrete(snapshot)
        if self.stream_pending_task is not None:
            self._tick_stream_pending(snapshot)

    def _process_connection(self, snapshot: DroneSnapshot) -> bool:
        if snapshot.failsafe:
            detail = snapshot.failsafe_reason.strip() or "原因未知"
            fault_msg = f"检测到 failsafe: {detail}"
            if (
                self.sm.state != LifecycleState.FAULT
                or self.ctx.current_error != fault_msg
            ):
                self._append_record("SYSTEM", "FAILED", fault_msg)
            self._to_fault(fault_msg)
            return True

        stale_ms = int(time.time() * 1000) - snapshot.updated_at_ms
        if snapshot.connected and stale_ms > self.connection_timeout_ms:
            snapshot.connected = False
            self._append_record("SYSTEM", "FAILED", f"飞控连接超时: {stale_ms}ms")
            self._to_fault(f"飞控连接超时: {stale_ms}ms")
            return True

        if self.sm.state == LifecycleState.FAULT:
            pending = self.task_adapter.pending_count()
            recoverable = not snapshot.failsafe and (
                snapshot.connected or pending > 0
            )
            if recoverable:
                ok, from_s, to_s, reason = self.sm.transit(
                    LifecycleState.READY, "故障条件解除，自动恢复"
                )
                if ok:
                    self.ctx.current_error = None
                    self.ctx.active_task = None
                    self._log_transition(from_s.value, to_s.value, reason)
            else:
                return True

        if self.sm.state == LifecycleState.WAITING_CONNECTION and (
            snapshot.connected or self.task_adapter.pending_count() > 0
        ):
            ok, from_s, to_s, reason = self.sm.transit(
                LifecycleState.READY,
                "飞控连接可用" if snapshot.connected else "有待执行任务",
            )
            if ok:
                self._log_transition(from_s.value, to_s.value, reason)

        return False

    def _process_stream(self, snapshot: DroneSnapshot) -> bool:
        # 无新任务时继续跟踪当前流式设定点。
        self._tick_stream_pending(snapshot)
        return True

    def _process_discrete(self, snapshot: DroneSnapshot) -> None:
        del snapshot
        if self.sm.state not in {LifecycleState.READY, LifecycleState.EXECUTING}:
            return
        if self.stream_pending_task is not None:
            return
        task = self.task_adapter.fetch_task()
        if task is None:
            return
        if not self._enter_executing(task, f"接收任务: {task.task_id}"):
            return
        self._execute_task_payload(task)

    def _enter_executing(self, task: TaskCommand, reason: str) -> bool:
        ok, from_s, to_s, msg = self.sm.transit(LifecycleState.EXECUTING, reason)
        if not ok:
            self.logger.warning(msg)
            self._append_record(task.task_id, "REJECTED", msg, task.task_type.value)
            return False
        self.ctx.active_task = task
        self._log_transition(from_s.value, to_s.value, msg)
        return True

    def _execute_task_payload(self, task: TaskCommand) -> None:
        try:
            if (
                task.task_type == TaskType.MODE_SWITCH
                and str(task.payload.get("mode", "")).upper() == "RETURN_HOME"
            ):
                self._set_rth_phase("RTH_START", "开始执行返航任务")
                self.px4_adapter.execute_return_home_and_land(phase_cb=self._set_rth_phase)
            elif task.task_type in self._STREAM_TYPES:
                self.px4_adapter.switch_to_offboard_and_confirm()
                self.px4_adapter.publish_task(task)
                self.stream_pending_task = task
                self.stream_pending_start_ms = int(time.time() * 1000)
                return
            else:
                if task.task_type == TaskType.ARMING:
                    self._ensure_posctl_before_arm_if_needed(task)
                self.px4_adapter.publish_task(task)

            if task.task_type in {TaskType.MODE_SWITCH, TaskType.ARMING} and str(
                task.payload.get("mode", "")
            ).upper() != "RETURN_HOME":
                command = self.px4_adapter.last_vehicle_command_sent()
                if command is None:
                    raise RuntimeError("命令发送后未获取到 command id")
                ack_ok, ack_detail = self.px4_adapter.wait_vehicle_command_ack(
                    command, timeout_ms=1000
                )
                if not ack_ok:
                    raise RuntimeError(f"命令 ACK 失败: {ack_detail}")
                if task.task_type == TaskType.ARMING and bool(task.payload.get("arm", False)):
                    self._capture_home_on_arm()

            ok, from_s, to_s, tr_reason = self.sm.transit(
                LifecycleState.READY, f"任务完成: {task.task_id}"
            )
            if ok:
                self._log_transition(from_s.value, to_s.value, tr_reason)
            self._append_record(task.task_id, "SUCCESS", "任务执行完成", task.task_type.value)
            self.ctx.active_task = None
            if (
                task.task_type == TaskType.MODE_SWITCH
                and str(task.payload.get("mode", "")).upper() == "RETURN_HOME"
            ):
                self._set_rth_phase("IDLE", "")
        except Exception as exc:  # pylint: disable=broad-except
            self._append_record(task.task_id, "FAILED", str(exc), task.task_type.value)
            if (
                task.task_type == TaskType.MODE_SWITCH
                and str(task.payload.get("mode", "")).upper() == "RETURN_HOME"
            ):
                self._set_rth_phase("RTH_FAILED", str(exc))
            self._to_fault(f"任务执行失败: {task.task_id}, err={exc}")

    def _ensure_posctl_before_arm_if_needed(self, task: TaskCommand) -> None:
        if not bool(task.payload.get("arm", False)):
            return
        if task.payload.get("confirm_current_mode") is True:
            return
        snap = self.px4_adapter.get_snapshot()
        if not snap.connected:
            raise RuntimeError(
                "飞控未连接，拒绝解锁；若需在非定点模式下解锁请设置 payload.confirm_current_mode=true"
            )
        if self.px4_adapter.is_navigation_state_posctl():
            return
        self.px4_adapter.switch_to_posctl_and_confirm()

    def _tick_stream_pending(self, snapshot: DroneSnapshot) -> None:
        task = self.stream_pending_task
        if task is None:
            return
        try:
            self.px4_adapter.publish_task(task)
        except Exception as exc:  # pylint: disable=broad-except
            self.stream_pending_task = None
            self._append_record(task.task_id, "FAILED", str(exc), task.task_type.value)
            self.ctx.active_task = None
            self._to_fault(f"流式任务发布失败: {task.task_id}, err={exc}")
            return

        now_ms = int(time.time() * 1000)
        elapsed = now_ms - self.stream_pending_start_ms

        if task.task_type == TaskType.POSITION_CONTROL:
            reached, detail = self._is_position_reached(task, snapshot)
            effective_deadline_ms = max(int(task.deadline_ms), 30_000)
            if reached:
                self._finish_stream(task, "SUCCESS", detail)
                return
            if elapsed >= effective_deadline_ms:
                tx = float(task.payload.get("x", 0.0))
                ty = float(task.payload.get("y", 0.0))
                tz = float(task.payload.get("z", 0.0))
                px, py, pz = snapshot.position
                horiz = math.hypot(tx - px, ty - py)
                dz = abs(tz - pz)
                self._finish_stream(
                    task,
                    "FAILED",
                    f"{task.task_type.value} 到达超时({effective_deadline_ms}ms), "
                    f"Δxy={horiz:.2f}m Δz={dz:.2f}m",
                )
            return

        if task.task_type == TaskType.VELOCITY_CONTROL:
            if elapsed >= task.deadline_ms:
                self._finish_stream(
                    task, "SUCCESS", f"流式控制已持续 {task.deadline_ms}ms"
                )

    def _is_position_reached(
        self, task: TaskCommand, snapshot: DroneSnapshot
    ) -> tuple[bool, str]:
        tx = float(task.payload.get("x", 0.0))
        ty = float(task.payload.get("y", 0.0))
        tz = float(task.payload.get("z", 0.0))
        px, py, pz = snapshot.position
        vx, vy, vz = snapshot.velocity
        horiz = math.hypot(tx - px, ty - py)
        dz = abs(tz - pz)
        speed = math.sqrt(vx * vx + vy * vy + vz * vz)
        r_xy = float(task.payload.get("arrival_radius_m", 1.0))
        r_z = float(task.payload.get("arrival_dz_m", 0.6))
        speed_threshold = float(task.payload.get("arrival_speed_mps", 0.5))
        reached = horiz <= r_xy and dz <= r_z and speed <= speed_threshold
        detail = f"已到达目标 xy={horiz:.2f}m z={dz:.2f}m |v|={speed:.2f}m/s"
        return reached, detail

    def _finish_stream(self, task: TaskCommand, status: str, detail: str) -> None:
        self.stream_pending_task = None
        ok, from_s, to_s, reason = self.sm.transit(
            LifecycleState.READY, f"任务{status}: {task.task_id}"
        )
        if ok:
            self._log_transition(from_s.value, to_s.value, reason)
        self._append_record(task.task_id, status, detail, task.task_type.value)
        self.ctx.active_task = None
        if status == "SUCCESS":
            self.logger.info("任务完成: %s (%s)", task.task_id, detail)

    def _abort_stream(self, reason: str, status: str = "FAILED") -> None:
        task = self.stream_pending_task
        if task is None:
            return
        self.stream_pending_task = None
        self._append_record(task.task_id, status, reason, task.task_type.value)
        self.ctx.active_task = None
        ok, from_s, to_s, msg = self.sm.transit(LifecycleState.READY, "流式任务被中止")
        if ok:
            self._log_transition(from_s.value, to_s.value, msg)

    def _to_fault(self, reason: str) -> None:
        self.stream_pending_task = None
        self.ctx.current_error = reason
        ok, from_s, to_s, tr_reason = self.sm.transit(LifecycleState.FAULT, reason)
        if ok:
            if self.sm.state == LifecycleState.FAULT and self.last_tr_reason == reason:
                return
            self.logger.error("进入故障态: %s", reason)
            self._log_transition(from_s.value, to_s.value, tr_reason)
            self.last_tr_reason = reason

    def _log_transition(self, src: str, dst: str, reason: str) -> None:
        self.logger.info("状态转换: %s -> %s, reason=%s", src, dst, reason)

    def runtime_status(self) -> dict:
        snapshot = self.px4_adapter.get_snapshot()
        active_task_id = self.ctx.active_task.task_id if self.ctx.active_task else None
        now_ms = int(time.time() * 1000)
        return {
            "lifecycle_state": self.ctx.lifecycle_state.value,
            "active_task_id": active_task_id,
            "error": (
                self.ctx.current_error
                if self.sm.state == LifecycleState.FAULT
                else ""
            ),
            "rth_phase": self.ctx.rth_phase,
            "rth_detail": self.ctx.rth_detail,
            "task_history": [self._history[-1]] if self._history else [],
            "realtime": self.realtime_controller.status(),
            "telemetry": self.px4_adapter.telemetry_debug(now_ms),
            "drone": {
                "connected": snapshot.connected,
                "mode": snapshot.mode,
                "armed": snapshot.armed,
                "position": list(snapshot.position),
                "velocity": list(snapshot.velocity),
                "attitude_quat": list(snapshot.attitude_quat),
                "battery_remaining": snapshot.battery_remaining,
                "battery_voltage": snapshot.battery_voltage,
                "failsafe": snapshot.failsafe,
                "failsafe_reason": snapshot.failsafe_reason,
                "home_valid": snapshot.home_valid,
                "home_position": list(snapshot.home_position),
                "updated_at_ms": snapshot.updated_at_ms,
            },
        }

    def _append_record(
        self,
        task_id: str,
        status: str,
        detail: str,
        task_type: str = "SYSTEM",
    ) -> None:
        self._history.append(
            {
                "task_id": task_id,
                "task_type": task_type,
                "status": status,
                "detail": detail,
                "timestamp_ms": int(time.time() * 1000),
            }
        )
        if len(self._history) > 50:
            self._history = self._history[-50:]

    def _set_rth_phase(self, phase: str, detail: str) -> None:
        self.ctx.rth_phase = phase
        self.ctx.rth_detail = detail

    def _capture_home_on_arm(self) -> None:
        if self.px4_adapter.capture_home_at_current_position():
            home = self.px4_adapter.get_snapshot().home_position
            self.logger.info(
                "解锁时记录 home 点: x=%.2f y=%.2f z=%.2f",
                home[0],
                home[1],
                home[2],
            )

    def _handle_armed_state_transitions(self, snapshot: DroneSnapshot) -> bool:
        armed_now = bool(snapshot.armed)
        if self._last_armed_state is None:
            self._last_armed_state = armed_now
            return False
        armed_transition = not self._last_armed_state and armed_now
        disarmed_transition = self._last_armed_state and not armed_now
        self._last_armed_state = armed_now

        if armed_transition:
            self._capture_home_on_arm()

        if not disarmed_transition:
            return False

        self.px4_adapter.clear_home_on_disarm()
        self.logger.info("检测到上锁，清理任务")
        if self.realtime_controller.enabled():
            self.realtime_controller.disable()
        self.stream_pending_task = None
        self.ctx.active_task = None
        self.task_adapter.clear()
        if self.sm.state == LifecycleState.EXECUTING:
            ok, from_s, to_s, reason = self.sm.transit(
                LifecycleState.READY, "上锁清理"
            )
            if ok:
                self._log_transition(from_s.value, to_s.value, reason)
        return True

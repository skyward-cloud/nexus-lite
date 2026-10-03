"""Realtime 控制器（输入缓存 + 使能 + 执行）。"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from threading import Lock
from typing import TYPE_CHECKING, Any, Callable, Dict, Optional
import time

from .models import LifecycleState
from .unification_adapter import PX4TopicsAdapter
from .lifecycle import Lifecycle
from .models import RuntimeContext


class RealtimeController:
    """高频实时控制器：线程安全指令缓存、使能开关与 PX4 发布。"""

    @dataclass
    class _RealtimeCommand:
        use_position: bool = False
        use_velocity: bool = False
        use_attitude: bool = False
        use_acceleration: bool = False
        use_jerk: bool = False
        use_yaw_rate: bool = False
        ax: float = 0.0
        ay: float = 0.0
        az: float = 0.0
        jerk_x: float = 0.0
        jerk_y: float = 0.0
        jerk_z: float = 0.0
        yaw_rate: float = 0.0
        x: float = 0.0
        y: float = 0.0
        z: float = 0.0
        vx: float = 0.0
        vy: float = 0.0
        vz: float = 0.0
        yaw: float = 0.0
        qw: float = 1.0
        qx: float = 0.0
        qy: float = 0.0
        qz: float = 0.0
        thrust_z: float = 0.5
        timestamp_ms: int = 0

    def __init__(
        self,
        px4_port: "PX4TopicsAdapter",
        timeout_ms: int = 300,
        max_speed_mps: float = 2.0,
    ) -> None:
        self._px4 = px4_port
        self._lock = Lock()
        self._latest: Optional[RealtimeController._RealtimeCommand] = None
        self._enabled = False
        self._timeout_ms = timeout_ms
        self._max_speed_mps = max_speed_mps
        self._last_action = "IDLE"
        self._offboard_ready = False

    def submit_from_dict(self, raw: Dict[str, Any]) -> _RealtimeCommand:
        use_position = bool(raw.get("use_position", False))
        use_velocity = bool(raw.get("use_velocity", False))
        use_attitude = bool(raw.get("use_attitude", False))
        use_acceleration = bool(raw.get("use_acceleration", False))
        use_jerk = bool(raw.get("use_jerk", False))
        use_yaw_rate = bool(raw.get("use_yaw_rate", False))
        if not (
            use_position
            or use_velocity
            or use_attitude
            or use_acceleration
            or use_jerk
            or use_yaw_rate
        ):
            raise ValueError(
                "至少需要启用一种控制: use_position/use_velocity/use_attitude/"
                "use_acceleration/use_jerk/use_yaw_rate"
            )

        cmd = self._RealtimeCommand(
            use_position=use_position,
            use_velocity=use_velocity,
            use_attitude=use_attitude,
            use_acceleration=use_acceleration,
            use_jerk=use_jerk,
            use_yaw_rate=use_yaw_rate,
            x=float(raw.get("x", 0.0)),
            y=float(raw.get("y", 0.0)),
            z=float(raw.get("z", 0.0)),
            vx=float(raw.get("vx", 0.0)),
            vy=float(raw.get("vy", 0.0)),
            vz=float(raw.get("vz", 0.0)),
            ax=float(raw.get("ax", 0.0)),
            ay=float(raw.get("ay", 0.0)),
            az=float(raw.get("az", 0.0)),
            jerk_x=float(raw.get("jerk_x", 0.0)),
            jerk_y=float(raw.get("jerk_y", 0.0)),
            jerk_z=float(raw.get("jerk_z", 0.0)),
            yaw_rate=float(raw.get("yaw_rate", 0.0)),
            yaw=float(raw.get("yaw", 0.0)),
            qw=float(raw.get("qw", 1.0)),
            qx=float(raw.get("qx", 0.0)),
            qy=float(raw.get("qy", 0.0)),
            qz=float(raw.get("qz", 0.0)),
            thrust_z=float(raw.get("thrust_z", 0.5)),
            timestamp_ms=int(raw.get("timestamp_ms", int(time.time() * 1000))),
        )
        with self._lock:
            self._latest = cmd
        return cmd

    def latest(self) -> Optional[_RealtimeCommand]:
        with self._lock:
            return self._latest

    def enable(self) -> None:
        self._enabled = True
        self._last_action = "ENABLED"

    def disable(self) -> None:
        self._enabled = False
        self._offboard_ready = False
        self._px4.publish_hover()
        self._last_action = "DISABLED_HOLD"

    def enabled(self) -> bool:
        return self._enabled

    def process_tick(
        self,
        sm: "Lifecycle",
        ctx: "RuntimeContext",
        *,
        stream_abort: Callable[[str], None],
        append_record: Callable[..., None],
        to_fault: Callable[[str], None],
        log_transition: Callable[[str, str, str], None],
    ) -> bool:
        """实时 tick。返回 True 表示本 tick 应提前结束。"""
        if not self._enabled:
            if sm.state == LifecycleState.EXECUTING and ctx.active_task is None:
                ok, from_s, to_s, reason = sm.transit(
                    LifecycleState.READY, "实时控制释放"
                )
                if ok:
                    log_transition(from_s.value, to_s.value, reason)
            return False

        stream_abort("实时控制接管，离散流式任务中止")

        if sm.state == LifecycleState.READY:
            ok, from_s, to_s, reason = sm.transit(
                LifecycleState.EXECUTING, "实时控制接管"
            )
            if ok:
                log_transition(from_s.value, to_s.value, reason)

        try:
            if not self._offboard_ready:
                self._px4.switch_to_offboard_and_confirm()
                self._offboard_ready = True
            if self.step():
                # 指令超时：退出实时控制，交还 READY
                append_record(
                    "REALTIME",
                    "TIMEOUT",
                    f"指令超时({self._timeout_ms}ms)，自动退出实时控制",
                    "REALTIME",
                )
                if sm.state == LifecycleState.EXECUTING:
                    ok, from_s, to_s, reason = sm.transit(
                        LifecycleState.READY, "实时控制超时退出"
                    )
                    if ok:
                        log_transition(from_s.value, to_s.value, reason)
                return False
        except Exception as exc:  # pylint: disable=broad-except
            append_record("REALTIME", "FAILED", str(exc), "REALTIME")
            to_fault(f"实时控制失败: err={exc}")

        return True

    def step(self) -> bool:
        """执行一步设定点。返回 True 表示因超时已退出实时控制。"""
        if not self._enabled:
            return False

        cmd = self.latest()
        now_ms = int(time.time() * 1000)

        if cmd is None:
            self._px4.publish_hover()
            self._last_action = "NO_CMD_HOLD"
            return False

        timed_out = now_ms - cmd.timestamp_ms > self._timeout_ms
        if timed_out:
            # 超时：锁存当前位置悬停后退出实时控制
            self._px4.publish_hover(recapture=True)
            self._enabled = False
            self._offboard_ready = False
            with self._lock:
                self._latest = None
            self._last_action = "CMD_TIMEOUT_EXIT"
            return True

        self._px4.clear_hover()

        vx = self._clamp(cmd.vx, -self._max_speed_mps, self._max_speed_mps)
        vy = self._clamp(cmd.vy, -self._max_speed_mps, self._max_speed_mps)
        vz = self._clamp(cmd.vz, -self._max_speed_mps, self._max_speed_mps)

        self._px4.publish_composite_setpoint(
            use_position=cmd.use_position,
            use_velocity=cmd.use_velocity,
            use_attitude=cmd.use_attitude,
            use_acceleration=cmd.use_acceleration,
            use_jerk=cmd.use_jerk,
            use_yaw_rate=cmd.use_yaw_rate,
            x=cmd.x,
            y=cmd.y,
            z=cmd.z,
            vx=vx,
            vy=vy,
            vz=vz,
            ax=cmd.ax,
            ay=cmd.ay,
            az=cmd.az,
            jerk_x=cmd.jerk_x,
            jerk_y=cmd.jerk_y,
            jerk_z=cmd.jerk_z,
            yaw_rate=cmd.yaw_rate,
            yaw=cmd.yaw,
            qw=cmd.qw,
            qx=cmd.qx,
            qy=cmd.qy,
            qz=cmd.qz,
            thrust_z=cmd.thrust_z,
        )
        self._last_action = "TRACKING"
        return False

    def status(self) -> Dict[str, Any]:
        latest = self.latest()
        return {
            "enabled": self._enabled,
            "timeout_ms": self._timeout_ms,
            "max_speed_mps": self._max_speed_mps,
            "last_action": self._last_action,
            "has_cmd": latest is not None,
            "latest_cmd": asdict(latest) if latest is not None else None,
        }

    @staticmethod
    def _clamp(v: float, low: float, high: float) -> float:
        return max(low, min(high, v))

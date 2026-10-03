"""外部任务接入：校验后只保留最后一条，供控制环取走执行。"""

from __future__ import annotations

from threading import Lock
from typing import Any, Dict, Optional
import time

from .models import TaskCommand, TaskType


class TaskServerAdapter:
    """任务入口：校验 payload，只保留最新一条待执行任务。"""

    def __init__(self) -> None:
        self._lock = Lock()
        self._pending: Optional[TaskCommand] = None
        self._last_task_id: Optional[str] = None
        self._last_submit_ts_ms: int = 0

    def submit(self, raw: Dict[str, Any]) -> TaskCommand:
        """校验字典并覆盖为最新任务；返回构建好的 TaskCommand。"""
        self._validate_task_payload(raw)
        task_type = TaskType(str(raw["task_type"]))
        task = TaskCommand(
            task_id=str(raw["task_id"]),
            task_type=task_type,
            payload=dict(raw.get("payload", {})),
            deadline_ms=int(raw.get("deadline_ms", _default_deadline_ms_for_task(task_type))),
        )
        with self._lock:
            self._pending = task
            self._last_task_id = task.task_id
            self._last_submit_ts_ms = int(time.time() * 1000)
        return task

    def fetch_task(self) -> Optional[TaskCommand]:
        """取出当前待执行任务；无则返回 None。"""
        with self._lock:
            task = self._pending
            self._pending = None
            return task

    def clear(self) -> None:
        with self._lock:
            self._pending = None

    def pending_count(self) -> int:
        with self._lock:
            return 0 if self._pending is None else 1

    def stats(self) -> Dict[str, Any]:
        return {
            "pending_count": self.pending_count(),
            "last_task_id": self._last_task_id,
            "last_submit_ts_ms": self._last_submit_ts_ms,
        }

    def _validate_task_payload(self, raw: Dict[str, Any]) -> None:
        required = {"task_id", "task_type", "payload"}
        missing = [k for k in required if k not in raw]
        if missing:
            raise KeyError(",".join(missing))

        task_type = TaskType(str(raw["task_type"]))
        payload = raw["payload"]
        if not isinstance(payload, dict):
            raise ValueError("payload 必须是对象")

        schema = {
            TaskType.POSITION_CONTROL: {"x", "y", "z"},
            TaskType.VELOCITY_CONTROL: {"vx", "vy", "vz"},
            TaskType.MODE_SWITCH: {"mode"},
            TaskType.ARMING: {"arm"},
            TaskType.KILL_SWITCH: set(),
        }
        required_payload = schema[task_type]
        payload_missing = [k for k in required_payload if k not in payload]
        if payload_missing:
            raise ValueError(f"payload 缺少字段: {payload_missing}")
        if task_type == TaskType.ARMING and "confirm_current_mode" in payload:
            if not isinstance(payload["confirm_current_mode"], bool):
                raise ValueError("confirm_current_mode 必须为布尔类型")
        if task_type == TaskType.MODE_SWITCH:
            mode_raw = str(payload.get("mode", "")).strip()
            allowed_mode = frozenset(
                {
                    "OFFBOARD",
                    "LAND",
                    "HOLD",
                    "RETURN_HOME",
                    "MANUAL",
                    "ALTCTL",
                    "ALTITUDE",
                    "POSCTL",
                    "POSITION",
                    "AUTO",
                    "ACRO",
                    "STABILIZED",
                    "STAB",
                    "RATTITUDE",
                }
            )
            mode_key = mode_raw.upper() if mode_raw.isascii() else mode_raw
            if mode_key not in allowed_mode:
                raise ValueError(
                    f"mode 不支持: {mode_raw}, 允许值: {sorted(allowed_mode)}"
                )


def _default_deadline_ms_for_task(task_type: TaskType) -> int:
    if task_type == TaskType.POSITION_CONTROL:
        return 1_800_000
    if task_type == TaskType.VELOCITY_CONTROL:
        return 10_000
    return 5_000

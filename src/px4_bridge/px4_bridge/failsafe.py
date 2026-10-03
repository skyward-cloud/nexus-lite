"""Failsafe 标志共享逻辑。"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

FAILSAFE_BOOL_FIELDS: tuple[str, ...] = (
    "angular_velocity_invalid",
    "attitude_invalid",
    "local_altitude_invalid",
    "local_position_invalid",
    "local_position_invalid_relaxed",
    "local_velocity_invalid",
    "home_position_invalid",
    "battery_low_remaining_time",
    "battery_unhealthy",
    "geofence_breached",
    "mission_failure",
    "vtol_fixed_wing_system_failure",
    "wind_limit_exceeded",
    "flight_time_limit_exceeded",
    "local_position_accuracy_low",
    "fd_critical_failure",
    "fd_esc_arming_failure",
    "fd_imbalanced_prop",
    "fd_motor_failure",
)

DEFAULT_FAILSAFE_FLAG_SKIP: tuple[str, ...] = (
    "home_position_invalid",
)


def failsafe_flag_skip_config_path() -> Path:
    """向上查找 workspace 的 config/failsafe_flag_skip.json。"""
    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / "config" / "failsafe_flag_skip.json"
        if candidate.is_file():
            return candidate
    return here.parents[4] / "config" / "failsafe_flag_skip.json"


def load_failsafe_flag_skip_from_config_file() -> tuple[str, ...]:
    """读取 failsafe_flag_skip.json；缺失或损坏时返回空元组。"""
    path = failsafe_flag_skip_config_path()
    if not path.is_file():
        return ()
    try:
        with path.open(encoding="utf-8") as f:
            data = json.load(f)
    except (OSError, json.JSONDecodeError):
        return ()
    if not isinstance(data, list):
        return ()
    return tuple(str(x) for x in data)


def failsafe_state_from_failsafe_flags(
    msg: Any,
    failsafe_flag_skip: frozenset[str],
) -> tuple[bool, str]:
    """根据 FailsafeFlags 消息计算是否 failsafe 及原因。"""
    parts: list[str] = []
    for name in FAILSAFE_BOOL_FIELDS:
        if name in failsafe_flag_skip:
            continue
        if getattr(msg, name, False) is True:
            parts.append(name)
    active = len(parts) > 0
    reason = "; ".join(parts) if active else ""
    return active, reason

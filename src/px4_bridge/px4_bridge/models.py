"""领域模型定义。"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, Optional
import time


class TaskType(str, Enum):
    """任务类型定义。"""

    POSITION_CONTROL = "POSITION_CONTROL"
    VELOCITY_CONTROL = "VELOCITY_CONTROL"
    MODE_SWITCH = "MODE_SWITCH"
    KILL_SWITCH = "KILL_SWITCH"
    ARMING = "ARMING"


class LifecycleState(str, Enum):
    """系统生命周期状态（lite：无 HOLDING）。"""

    INIT = "INIT"
    WAITING_CONNECTION = "WAITING_CONNECTION"
    READY = "READY"
    EXECUTING = "EXECUTING"
    FAULT = "FAULT"
    SHUTDOWN = "SHUTDOWN"


@dataclass
class TaskCommand:
    """外部任务输入的统一结构。"""

    task_id: str
    task_type: TaskType
    payload: Dict[str, Any]
    deadline_ms: int = 5000
    created_at_ms: int = field(default_factory=lambda: int(time.time() * 1000))


@dataclass
class DroneSnapshot:
    """无人机状态快照（内部与对外均为 ENU；姿态为 FLU）。"""

    connected: bool = False
    mode: str = "UNKNOWN"
    armed: bool = False
    position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    velocity: tuple[float, float, float] = (0.0, 0.0, 0.0)
    attitude_quat: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    battery_remaining: float = 1.0
    battery_voltage: float = 0.0
    failsafe: bool = False
    failsafe_reason: str = ""
    home_valid: bool = False
    home_position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    home_updated_at_ms: int = 0
    home_lat_deg: float = 0.0
    home_lon_deg: float = 0.0
    home_alt_msl_m: float = 0.0
    updated_at_ms: int = field(default_factory=lambda: int(time.time() * 1000))
    local_position_xy_valid: bool = False
    local_position_z_valid: bool = False
    local_velocity_xy_valid: bool = False
    local_velocity_z_valid: bool = False
    global_lat_deg: float = 0.0
    global_lon_deg: float = 0.0
    global_position_valid: bool = False


@dataclass
class RuntimeContext:
    """运行时上下文。"""

    lifecycle_state: LifecycleState = LifecycleState.INIT
    active_task: Optional[TaskCommand] = None
    current_error: Optional[str] = None
    rth_phase: str = "IDLE"
    rth_detail: str = ""

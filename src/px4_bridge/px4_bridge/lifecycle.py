"""极简生命周期状态机。"""

from __future__ import annotations

from .models import LifecycleState, RuntimeContext

TransitionTuple = tuple[bool, LifecycleState, LifecycleState, str]


class Lifecycle:
    """lite 生命周期：INIT → WAITING_CONNECTION → READY ↔ EXECUTING，加 FAULT/SHUTDOWN。"""

    def __init__(self, context: RuntimeContext | None = None) -> None:
        self._state = LifecycleState.INIT
        self.context = context or RuntimeContext(lifecycle_state=self._state)

    @property
    def state(self) -> LifecycleState:
        return self._state

    def transit(self, to_state: LifecycleState, reason: str = "") -> TransitionTuple:
        if not self._can_transit(self._state, to_state):
            return (
                False,
                self._state,
                self._state,
                f"非法状态转换: {self._state} -> {to_state}",
            )
        from_state = self._state
        self._state = to_state
        self.context.lifecycle_state = to_state
        return (True, from_state, to_state, reason)

    def _can_transit(self, src: LifecycleState, dst: LifecycleState) -> bool:
        if src == dst:
            return True
        allowed = {
            LifecycleState.INIT: {
                LifecycleState.WAITING_CONNECTION,
                LifecycleState.SHUTDOWN,
            },
            LifecycleState.WAITING_CONNECTION: {
                LifecycleState.READY,
                LifecycleState.FAULT,
                LifecycleState.SHUTDOWN,
            },
            LifecycleState.READY: {
                LifecycleState.EXECUTING,
                LifecycleState.FAULT,
                LifecycleState.SHUTDOWN,
            },
            LifecycleState.EXECUTING: {
                LifecycleState.READY,
                LifecycleState.FAULT,
                LifecycleState.SHUTDOWN,
            },
            LifecycleState.FAULT: {
                LifecycleState.READY,
                LifecycleState.SHUTDOWN,
            },
            LifecycleState.SHUTDOWN: set(),
        }
        return dst in allowed[src]

"""px4_bridge 行为配置加载。"""

from __future__ import annotations

import os
from pathlib import Path
from types import SimpleNamespace
from typing import Any

try:
    import yaml
except ImportError:
    yaml = None  # type: ignore[assignment]


def _config_path() -> Path:
    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / "config" / "bridge.yaml"
        if candidate.is_file():
            return candidate
    return here.parents[3] / "config" / "bridge.yaml"


def load_bridge_config() -> SimpleNamespace:
    """读取 config/bridge.yaml 与环境变量。"""
    defaults = SimpleNamespace(
        loop_hz=20,
        connection_timeout_ms=1000,
        realtime_timeout_ms=300,
        realtime_max_speed_mps=2.0,
        enable_file_logging=False,
        log_file_path="logs/px4_bridge.log",
    )
    path = _config_path()
    cfg = defaults
    if path.is_file() and yaml is not None:
        try:
            with path.open(encoding="utf-8") as f:
                raw: Any = yaml.safe_load(f)
            if isinstance(raw, dict):
                cfg = SimpleNamespace(
                    loop_hz=int(raw.get("loop_hz", defaults.loop_hz)),
                    connection_timeout_ms=int(
                        raw.get("connection_timeout_ms", defaults.connection_timeout_ms)
                    ),
                    realtime_timeout_ms=int(
                        raw.get("realtime_timeout_ms", defaults.realtime_timeout_ms)
                    ),
                    realtime_max_speed_mps=float(
                        raw.get("realtime_max_speed_mps", defaults.realtime_max_speed_mps)
                    ),
                    enable_file_logging=defaults.enable_file_logging,
                    log_file_path=defaults.log_file_path,
                )
        except (OSError, yaml.YAMLError):
            cfg = defaults

    loop_hz = os.getenv("PX4_BRIDGE_LOOP_HZ", "").strip()
    if loop_hz:
        cfg.loop_hz = int(loop_hz)
    timeout = os.getenv("PX4_BRIDGE_CONNECTION_TIMEOUT_MS", "").strip()
    if timeout:
        cfg.connection_timeout_ms = int(timeout)
    file_log = os.getenv("PX4_BRIDGE_FILE_LOGGING", "").strip().lower()
    if file_log in {"1", "true", "yes", "on"}:
        cfg.enable_file_logging = True
    log_path = os.getenv("PX4_BRIDGE_LOG_FILE", "").strip()
    if log_path:
        cfg.log_file_path = log_path
    return cfg

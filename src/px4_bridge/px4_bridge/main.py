"""程序入口：装配配置、控制环，并提供 console_scripts。"""

from __future__ import annotations

import logging
import signal
from typing import Optional

from .bridge_config import load_bridge_config
from .node import PX4BridgeNode


def setup_logging() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s [%(name)s] %(message)s",
    )


class BridgeApplication:
    """lite 进程组合根：配置 + PX4BridgeNode + 信号处理。"""

    def __init__(self, config: Optional[object] = None) -> None:
        self._config = config or load_bridge_config()
        self._logger = logging.getLogger(self.__class__.__name__)
        self._stopped = False
        self.node = PX4BridgeNode.create_default(
            loop_hz=int(self._config.loop_hz),
            connection_timeout_ms=int(self._config.connection_timeout_ms),
            enable_file_logging=bool(self._config.enable_file_logging),
            log_file_path=str(self._config.log_file_path),
            realtime_timeout_ms=int(self._config.realtime_timeout_ms),
            realtime_max_speed_mps=float(self._config.realtime_max_speed_mps),
        )

    def stop_all(self) -> None:
        if self._stopped:
            return
        self._stopped = True
        self.node.stop()

    def run(self) -> int:
        def _handle_signal(sig: int, _frame: object) -> None:
            self._logger.info("收到信号: %s，准备退出", sig)
            # 仅打断主循环；资源释放交给 finally，避免 stop 执行两次。
            self.node.request_stop()

        signal.signal(signal.SIGINT, _handle_signal)
        signal.signal(signal.SIGTERM, _handle_signal)

        exit_code = 0
        try:
            self.node.start()
        except KeyboardInterrupt:
            pass
        except Exception as exc:  # pylint: disable=broad-except
            self._logger.exception("主程序异常: %s", exc)
            exit_code = 1
        finally:
            self.stop_all()
        return exit_code


def main(args: list[str] | None = None) -> int:
    del args
    setup_logging()
    return BridgeApplication().run()


def odom_bridge_main(args: list[str] | None = None) -> int:
    del args
    from .odom_bridge_node import main as odom_main

    return odom_main()


if __name__ == "__main__":
    raise SystemExit(main())

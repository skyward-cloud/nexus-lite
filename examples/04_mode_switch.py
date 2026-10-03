#!/usr/bin/env python3
"""示例 4：模式切换与状态打印。

仅演示 MODE_SWITCH / 读 status，默认不解锁起飞（安全）。

运行：
  python3 examples/04_mode_switch.py
  python3 examples/04_mode_switch.py --to OFFBOARD
  python3 examples/04_mode_switch.py --to POSCTL
"""

from __future__ import annotations

import argparse
import time

from common import make_client


def main() -> int:
    parser = argparse.ArgumentParser(description="模式切换示例")
    parser.add_argument(
        "--to",
        default="POSCTL",
        help="目标模式：POSCTL/OFFBOARD/HOLD/LAND/MANUAL/ALTCTL/...",
    )
    parser.add_argument("--watch-s", type=float, default=3.0, help="切换后观察秒数")
    args = parser.parse_args()

    client = make_client("example_mode_switch")
    try:
        st = client.wait_status()
        client.get_logger().info(
            "当前状态: "
            f"life={st.lifecycle_state} connected={st.connected} "
            f"mode={st.mode} armed={st.armed} "
            f"pos=[{st.position[0]:.2f},{st.position[1]:.2f},{st.position[2]:.2f}] "
            f"batt={st.battery_remaining:.2f} failsafe={st.failsafe}"
        )

        client.submit_task(
            f"ex4-mode-{args.to.lower()}",
            "MODE_SWITCH",
            {"mode": str(args.to).upper()},
        )
        end = time.time() + args.watch_s
        while time.time() < end:
            st = client.status()
            client.get_logger().info(
                f"life={st.lifecycle_state} mode={st.mode} armed={st.armed} "
                f"active={st.active_task_id!r} err={st.error!r}"
            )
            time.sleep(0.5)
        return 0
    finally:
        client.destroy_node()


if __name__ == "__main__":
    raise SystemExit(main())

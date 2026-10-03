#!/usr/bin/env python3
"""示例 2：指点飞行（POSITION_CONTROL）。

默认流程：解锁 → 起飞 → 飞到目标点 → 降落。
坐标系：局部 ENU，z 向上为正。

运行：
  python3 examples/02_position_point.py --x 2 --y 0 --z 1.5
"""

from __future__ import annotations

import argparse
import time

from common import make_client


def main() -> int:
    parser = argparse.ArgumentParser(description="指点飞行示例（POSITION_CONTROL）")
    parser.add_argument("--x", type=float, default=2.0)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--z", type=float, default=1.5, help="ENU 高度，正=向上")
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--takeoff-z", type=float, default=2.0)
    parser.add_argument("--arrival-xy", type=float, default=0.3, help="到点水平半径 (m)")
    parser.add_argument("--arrival-dz", type=float, default=0.2, help="到点高度容差 (m)")
    parser.add_argument("--arrival-speed", type=float, default=0.5, help="到点速度阈值 (m/s)")
    parser.add_argument("--skip-arm", action="store_true", help="已解锁时跳过 ARMING")
    parser.add_argument("--skip-land", action="store_true", help="结束后不降落")
    args = parser.parse_args()

    takeoff = (0.0, 0.0, float(args.takeoff_z))
    point = (float(args.x), float(args.y), float(args.z))
    arrival = {
        "arrival_radius_m": float(args.arrival_xy),
        "arrival_dz_m": float(args.arrival_dz),
        "arrival_speed_mps": float(args.arrival_speed),
    }
    arrived_kw = {
        "radius_m": float(args.arrival_xy),
        "dz_m": float(args.arrival_dz),
        "speed_mps": float(args.arrival_speed),
        "timeout_s": 10.0,
    }

    client = make_client("example_position_point")
    try:
        client.wait_status()
        client.publish_realtime(enable=False, has_command=False)
        client.spin_brief(0.2)

        if not args.skip_arm:
            client.submit_task("ex2-arm", "ARMING", {"arm": True})
            client.wait_task_idle(timeout_s=15.0, after_task_id="ex2-arm")
            client.wait_armed(True, timeout_s=10.0)

        client.submit_task(
            "ex2-takeoff",
            "POSITION_CONTROL",
            {
                "x": takeoff[0],
                "y": takeoff[1],
                "z": takeoff[2],
                "yaw": float(args.yaw),
                **arrival,
            },
            deadline_ms=60_000,
        )
        client.wait_task_idle(timeout_s=90.0, after_task_id="ex2-takeoff")
        st = client.wait_until_arrived(*takeoff, **arrived_kw)
        client.get_logger().info(f"起飞到点: pos={list(st.position)}")

        client.submit_task(
            "ex2-point",
            "POSITION_CONTROL",
            {
                "x": point[0],
                "y": point[1],
                "z": point[2],
                "yaw": float(args.yaw),
                **arrival,
            },
            deadline_ms=120_000,
        )
        client.wait_task_idle(timeout_s=150.0, after_task_id="ex2-point")
        st = client.wait_until_arrived(*point, **arrived_kw)
        client.get_logger().info(f"指点到点: pos={list(st.position)}")

        if not args.skip_land:
            client.submit_task("ex2-land", "MODE_SWITCH", {"mode": "LAND"})
            time.sleep(8.0)
            client.submit_task("ex2-disarm", "ARMING", {"arm": False})
        return 0
    finally:
        client.destroy_node()


if __name__ == "__main__":
    raise SystemExit(main())

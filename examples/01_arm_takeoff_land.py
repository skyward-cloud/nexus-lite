#!/usr/bin/env python3
"""示例 1：解锁 → 起飞到 1m → 悬停片刻 → 降落 → 上锁。

坐标系：局部 ENU，z 向上为正（起飞 z=2.0 约升高 2m）。

前置：
  ros2 launch px4_bridge bridge.launch.py
  # 以及 MicroXRCE-DDS-Agent + PX4（SITL/真机）

运行：
  source install/setup.bash
  python3 examples/01_arm_takeoff_land.py
"""

from __future__ import annotations

import argparse
import time

from common import make_client


def main() -> int:
    parser = argparse.ArgumentParser(description="解锁-起飞-降落示例")
    parser.add_argument("--takeoff-z", type=float, default=2.0, help="起飞高度")
    parser.add_argument("--hover-s", type=float, default=5.0, help="到点后悬停秒数")
    parser.add_argument("--arrival-xy", type=float, default=0.3, help="到点水平半径 (m)")
    parser.add_argument("--arrival-dz", type=float, default=0.2, help="到点高度容差 (m)")
    parser.add_argument("--arrival-speed", type=float, default=0.5, help="到点速度阈值 (m/s)")
    args = parser.parse_args()

    target = (0.0, 0.0, float(args.takeoff_z))
    arrival = {
        "arrival_radius_m": float(args.arrival_xy),
        "arrival_dz_m": float(args.arrival_dz),
        "arrival_speed_mps": float(args.arrival_speed),
    }

    client = make_client("example_arm_takeoff_land")
    try:
        st = client.wait_status()
        client.publish_realtime(enable=False, has_command=False)
        client.spin_brief(0.2)
        client.get_logger().info(
            f"connected={st.connected} mode={st.mode} armed={st.armed} life={st.lifecycle_state}"
        )

        # 1) 解锁（默认要求 POSCTL；SITL 通常已在定点）
        client.submit_task("ex1-arm", "ARMING", {"arm": True})
        client.wait_task_idle(timeout_s=15.0, after_task_id="ex1-arm")
        client.wait_armed(True, timeout_s=10.0)

        # 2) 起飞（POSITION_CONTROL，流式到点）
        client.submit_task(
            "ex1-takeoff",
            "POSITION_CONTROL",
            {
                "x": target[0],
                "y": target[1],
                "z": target[2],
                "yaw": 0.0,
                **arrival,
            },
            deadline_ms=60_000,
        )
        # 先等桥侧任务结束，再按位置复核到点（避免 submit 后误判空闲）
        client.wait_task_idle(timeout_s=90.0, after_task_id="ex1-takeoff")
        st = client.wait_until_arrived(
            *target,
            radius_m=float(args.arrival_xy),
            dz_m=float(args.arrival_dz),
            speed_mps=float(args.arrival_speed),
            timeout_s=10.0,
        )
        client.get_logger().info(f"到点: pos={list(st.position)}")
        client.get_logger().info(f"悬停 {args.hover_s:.1f}s")
        time.sleep(args.hover_s)

        # 3) 降落
        client.submit_task("ex1-land", "MODE_SWITCH", {"mode": "LAND"})
        time.sleep(8.0)

        # 4) 上锁
        client.submit_task("ex1-disarm", "ARMING", {"arm": False})
        time.sleep(1.0)
        st = client.status()
        client.get_logger().info(
            f"完成: armed={st.armed} mode={st.mode} pos={list(st.position)}"
        )
        return 0
    finally:
        client.destroy_node()


if __name__ == "__main__":
    raise SystemExit(main())

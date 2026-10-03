#!/usr/bin/env python3
"""示例 3：实时速度控制（RealtimeControl）。

流程：解锁 → 起飞 → 开启实时速度控制画圆若干秒 → 关闭实时 → 降落。
控制类型由 use_position / use_velocity / use_attitude 指定。

运行：
  python3 examples/03_realtime_velocity.py
  python3 examples/03_realtime_velocity.py --radius 1.5 --duration 12
"""

from __future__ import annotations

import argparse
import math
import time

from common import make_client


def main() -> int:
    parser = argparse.ArgumentParser(description="实时速度控制示例")
    parser.add_argument("--takeoff-z", type=float, default=2.0)
    parser.add_argument("--radius", type=float, default=1.0, help="水平圆半径 (m)")
    parser.add_argument("--speed", type=float, default=0.6, help="切向速度 (m/s)")
    parser.add_argument("--duration", type=float, default=10.0, help="画圆时长 (s)")
    parser.add_argument("--rate-hz", type=float, default=20.0, help="实时指令频率")
    parser.add_argument("--alt-kp", type=float, default=1.2, help="高度保持 P 增益 (vz = kp*(z_ref-z))")
    parser.add_argument("--max-vz", type=float, default=0.8, help="高度修正速度限幅 (m/s)")
    parser.add_argument("--arrival-xy", type=float, default=0.3, help="到点水平半径 (m)")
    parser.add_argument("--arrival-dz", type=float, default=0.2, help="到点高度容差 (m)")
    parser.add_argument("--arrival-speed", type=float, default=0.5, help="到点速度阈值 (m/s)")
    args = parser.parse_args()

    takeoff = (0.0, 0.0, float(args.takeoff_z))
    arrival = {
        "arrival_radius_m": float(args.arrival_xy),
        "arrival_dz_m": float(args.arrival_dz),
        "arrival_speed_mps": float(args.arrival_speed),
    }

    client = make_client("example_realtime_velocity")
    try:
        client.wait_status()
        # 上次若 Ctrl+C 中断，桥上 realtime 可能仍使能并抢占任务；先关掉
        client.publish_realtime(enable=False, has_command=False)
        time.sleep(0.3)

        client.submit_task("ex3-arm", "ARMING", {"arm": True})
        client.wait_task_idle(timeout_s=15.0, after_task_id="ex3-arm")
        client.wait_armed(True, timeout_s=10.0)

        client.submit_task(
            "ex3-takeoff",
            "POSITION_CONTROL",
            {
                "x": takeoff[0],
                "y": takeoff[1],
                "z": takeoff[2],
                "yaw": 0.0,
                **arrival,
            },
            deadline_ms=60_000,
        )
        client.wait_task_idle(timeout_s=90.0, after_task_id="ex3-takeoff")
        st = client.wait_until_arrived(
            *takeoff,
            radius_m=float(args.arrival_xy),
            dz_m=float(args.arrival_dz),
            speed_mps=float(args.arrival_speed),
            timeout_s=10.0,
        )
        client.get_logger().info(f"起飞到点: pos={list(st.position)}")

        # 以当前位置为圆心；锁定当前高度，用 vz P 控制抗掉高
        cx, cy = float(st.position[0]), float(st.position[1])
        z_ref = float(st.position[2])
        omega = args.speed / max(args.radius, 1e-3)
        dt = 1.0 / max(args.rate_hz, 1.0)
        max_vz = abs(float(args.max_vz))
        t0 = time.time()
        client.get_logger().info(
            f"实时画圆: center=({cx:.2f},{cy:.2f}) z_ref={z_ref:.2f} "
            f"r={args.radius} v={args.speed} alt_kp={args.alt_kp}"
        )

        while time.time() - t0 < args.duration:
            t = time.time() - t0
            # 切向速度：逆时针圆
            yaw = omega * t
            vx = -args.speed * math.sin(yaw)
            vy = args.speed * math.cos(yaw)
            # ENU：z 向上为正；掉高时 z 变小，需正 vz 爬升
            z_now = float(client.status().position[2])
            vz = float(args.alt_kp) * (z_ref - z_now)
            vz = max(-max_vz, min(max_vz, vz))
            client.publish_realtime(
                enable=True,
                has_command=True,
                use_velocity=True,
                vx=vx,
                vy=vy,
                vz=vz,
                yaw=yaw + math.pi / 2,  # 机头沿切线（ENU）
            )
            client.spin_brief(dt)

        # 关闭实时 → 零速 hold
        client.publish_realtime(enable=False, has_command=False)
        time.sleep(1.0)

        client.submit_task("ex3-land", "MODE_SWITCH", {"mode": "LAND"})
        client.wait_task_idle(timeout_s=30.0, after_task_id="ex3-land")
        client.submit_task("ex3-disarm", "ARMING", {"arm": False})
        client.wait_task_idle(timeout_s=15.0, after_task_id="ex3-disarm")
        return 0
    finally:
        try:
            client.publish_realtime(enable=False, has_command=False)
            client.spin_brief(0.2)
        except Exception:  # pylint: disable=broad-except
            pass
        client.destroy_node()


if __name__ == "__main__":
    raise SystemExit(main())

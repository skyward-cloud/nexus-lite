#!/usr/bin/env python3
"""示例 5：键盘控制（解锁 / 起飞 / 位置积分遥操 / 降落 / 上锁）。

必须在真实终端运行。订阅 /px4_bridge/out/status 获取无人机位姿，
经 TaskCommand 下发位置（不使用 RealtimeControl）。

长按飞行键：以 status 当前位置为基准，按速度前馈目标点并持续下发；松开即停。

任务按键（点按一次）：
  1 / B   解锁
  2 / T   起飞到 --takeoff-z（当前 xy，固定 yaw）
  3 / L   降落
  4 / U   上锁
  空格    悬停：设定点拉回 status 当前位置

飞行按键（机体水平系，机头前方为正；按住连续移动）：
  W/S     前 / 后
  A/D     左 / 右
  R/F     上 / 下
  Q/E     偏航左转 / 右转

  X / Esc 退出（不自动降落）

运行：
  python3 examples/05_keyboard_teleop.py
  python3 examples/05_keyboard_teleop.py --speed 0.8 --takeoff-z 1.2
"""

from __future__ import annotations

import argparse
import math
import select
import sys
import termios
import time
import tty
from typing import Optional

from common import BridgeClient, make_client
from px4_bridge_msgs.msg import BridgeStatus


def _yaw_from_quat(qw: float, qx: float, qy: float, qz: float) -> float:
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def _wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class Keyboard:
    """非阻塞终端按键读取（需 TTY）。"""

    def __init__(self) -> None:
        if not sys.stdin.isatty():
            raise RuntimeError("需要交互终端（TTY），请直接在终端运行本脚本")
        self._fd = sys.stdin.fileno()
        self._old = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)

    def close(self) -> None:
        termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)

    def read_key(self) -> Optional[str]:
        ready, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not ready:
            return None
        ch = sys.stdin.read(1)
        if ch == "\x1b":
            while select.select([sys.stdin], [], [], 0.0)[0]:
                sys.stdin.read(1)
            return "\x1b"
        return ch


def _pose_from_status(st: BridgeStatus) -> tuple[float, float, float, float]:
    """从 /px4_bridge/out/status 解析 ENU 位置与偏航。"""
    q = st.attitude_quat
    yaw = _yaw_from_quat(float(q[0]), float(q[1]), float(q[2]), float(q[3]))
    return (
        float(st.position[0]),
        float(st.position[1]),
        float(st.position[2]),
        yaw,
    )


def _wait_fresh_pose(
    client: BridgeClient, timeout_s: float = 2.0
) -> tuple[float, float, float, float]:
    """自旋等待最新 status，返回位姿。"""
    end = time.time() + timeout_s
    while time.time() < end:
        client.spin_brief(0.02)
        st = client.latest_status()
        if st is not None and bool(st.connected):
            return _pose_from_status(st)
    st = client.wait_status(timeout_s=timeout_s)
    return _pose_from_status(st)


def _submit_position(
    client: BridgeClient,
    *,
    task_id: str,
    x: float,
    y: float,
    z: float,
    yaw: float,
) -> None:
    """下发一次位置任务，不等待到达。"""
    client.submit_task(
        task_id,
        "POSITION_CONTROL",
        {
            "x": x,
            "y": y,
            "z": z,
            "yaw": yaw,
            "arrival_radius_m": 0.3,
            "arrival_dz_m": 0.2,
            "arrival_speed_mps": 0.5,
        },
        deadline_ms=60_000,
    )


def do_arm(client: BridgeClient) -> None:
    client.submit_task("kb-arm", "ARMING", {"arm": True})
    client.get_logger().info("解锁任务已提交")


def do_disarm(client: BridgeClient) -> None:
    client.submit_task("kb-disarm", "ARMING", {"arm": False})
    client.get_logger().info("上锁任务已提交")


def do_takeoff(client: BridgeClient, takeoff_z: float) -> None:
    x, y, _, yaw = _wait_fresh_pose(client)
    z = float(takeoff_z)
    _submit_position(client, task_id="kb-takeoff", x=x, y=y, z=z, yaw=yaw)
    client.get_logger().info(f"起飞任务已提交 -> ({x:.2f},{y:.2f},{z:.2f})")


def do_land(client: BridgeClient) -> None:
    client.submit_task("kb-land", "MODE_SWITCH", {"mode": "LAND"})
    client.get_logger().info("降落任务已提交")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="键盘控制：基于 /px4_bridge/out/status 位姿遥操"
    )
    parser.add_argument("--takeoff-z", type=float, default=1.2, help="起飞高度")
    parser.add_argument("--speed", type=float, default=1.5, help="水平速度 (m/s)")
    parser.add_argument("--vz-speed", type=float, default=0.4, help="升降速度 (m/s)")
    parser.add_argument("--yaw-rate", type=float, default=0.6, help="偏航角速度 (rad/s)")
    parser.add_argument("--rate-hz", type=float, default=20.0, help="积分/下发频率")
    parser.add_argument(
        "--lookahead-s",
        type=float,
        default=0.5,
        help="相对 status 当前位置的前馈时间 (s)",
    )
    parser.add_argument(
        "--key-hold-ms",
        type=float,
        default=220.0,
        help="按键松开判定延迟（终端无 key-up，靠超时）",
    )
    args = parser.parse_args()

    dt = 1.0 / max(args.rate_hz, 1.0)
    hold_s = max(args.key_hold_ms, 50.0) / 1000.0
    look_s = max(args.lookahead_s, 0.1)
    last: dict[str, float] = {
        "fwd": 0.0,
        "back": 0.0,
        "left": 0.0,
        "right": 0.0,
        "up": 0.0,
        "down": 0.0,
        "yaw_l": 0.0,
        "yaw_r": 0.0,
    }
    mapping = {
        "w": "fwd",
        "s": "back",
        "a": "left",
        "d": "right",
        "r": "up",
        "f": "down",
        "q": "yaw_l",
        "e": "yaw_r",
    }

    client = make_client("example_keyboard_teleop")
    kb: Optional[Keyboard] = None
    # 偏航指令可积分；位置每拍从 status 重算，避免虚拟点漂移
    cmd_yaw: Optional[float] = None
    pos_seq = 0

    try:
        st0 = client.wait_status()
        px, py, pz, yaw0 = _pose_from_status(st0)
        cmd_yaw = yaw0
        client.get_logger().info(
            f"已订阅 /px4_bridge/out/status  "
            f"connected={st0.connected} pos=({px:.2f},{py:.2f},{pz:.2f})"
        )

        kb = Keyboard()
        print(
            "\n键盘控制已就绪（位姿来自 /px4_bridge/out/status）\n"
            "  1/B 解锁   2/T 起飞   3/L 降落   4/U 上锁\n"
            "  W/S 前/后  A/D 左/右  R/F 上/下  Q/E 偏航（按住连续飞）\n"
            "  空格：设定点拉回当前位置\n"
            "  X/Esc 退出\n",
            flush=True,
        )

        running = True
        while running:
            # 持续接收 status
            client.spin_brief(dt)
            now = time.time()
            pending_action: Optional[str] = None
            snap_hover = False

            while True:
                key = kb.read_key()
                if key is None:
                    break
                k = key.lower() if key.isascii() and len(key) == 1 else key
                if k in {"x", "\x1b"}:
                    running = False
                    break
                if k in {"1", "b"}:
                    pending_action = "arm"
                    break
                if k in {"2", "t"}:
                    pending_action = "takeoff"
                    break
                if k in {"3", "l"}:
                    pending_action = "land"
                    break
                if k in {"4", "u"}:
                    pending_action = "disarm"
                    break
                if k == " ":
                    for ax in last:
                        last[ax] = 0.0
                    snap_hover = True
                    continue
                if k in mapping:
                    last[mapping[k]] = now

            if not running:
                break

            if pending_action is not None:
                for ax in last:
                    last[ax] = 0.0
                try:
                    if pending_action == "arm":
                        do_arm(client)
                    elif pending_action == "takeoff":
                        do_takeoff(client, args.takeoff_z)
                        _, _, _, cmd_yaw = _wait_fresh_pose(client)
                    elif pending_action == "land":
                        do_land(client)
                    elif pending_action == "disarm":
                        do_disarm(client)
                except Exception as exc:  # pylint: disable=broad-except
                    client.get_logger().error(f"{pending_action} 失败: {exc}")
                continue

            st = client.latest_status()
            if st is None:
                continue
            cur_x, cur_y, cur_z, cur_yaw = _pose_from_status(st)
            if cmd_yaw is None:
                cmd_yaw = cur_yaw

            def active(name: str) -> bool:
                t = last[name]
                return t > 0.0 and (now - t) <= hold_s

            if snap_hover:
                cmd_yaw = cur_yaw
                pos_seq += 1
                _submit_position(
                    client,
                    task_id=f"kb-pos-{pos_seq}",
                    x=cur_x,
                    y=cur_y,
                    z=cur_z,
                    yaw=cmd_yaw,
                )
                client.get_logger().info(
                    f"悬停 status=({cur_x:.2f},{cur_y:.2f},{cur_z:.2f})"
                )
                continue

            moving = any(active(n) for n in last)
            if not moving:
                continue

            v_fwd = (args.speed if active("fwd") else 0.0) + (
                -args.speed if active("back") else 0.0
            )
            v_right = (-args.speed if active("right") else 0.0) + (
                args.speed if active("left") else 0.0
            )
            v_up = (args.vz_speed if active("up") else 0.0) + (
                -args.vz_speed if active("down") else 0.0
            )
            if active("yaw_l"):
                cmd_yaw = _wrap_pi(cmd_yaw + args.yaw_rate * dt)
            if active("yaw_r"):
                cmd_yaw = _wrap_pi(cmd_yaw - args.yaw_rate * dt)

            # 以 status 当前位置为基准前馈目标，闭环跟随
            c, s = math.cos(cmd_yaw), math.sin(cmd_yaw)
            cmd_x = cur_x + (c * v_fwd - s * v_right) * look_s
            cmd_y = cur_y + (s * v_fwd + c * v_right) * look_s
            cmd_z = cur_z + v_up * look_s

            pos_seq += 1
            _submit_position(
                client,
                task_id=f"kb-pos-{pos_seq}",
                x=cmd_x,
                y=cmd_y,
                z=cmd_z,
                yaw=cmd_yaw,
            )

        return 0
    finally:
        if kb is not None:
            kb.close()
        client.destroy_node()


if __name__ == "__main__":
    raise SystemExit(main())

"""将 ROS2 Odometry 转为 PX4 VehicleOdometry 并发布到 visual_odometry 话题。"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any, Sequence

import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node
from rclpy.qos import QoSProfile

ROS2_AVAILABLE = True



@dataclass
class OdomBridgeConfig:
    """桥接节点配置（程序内变量）。"""

    input_topic: str = "/drone260/odom"
    output_topic: str = "/fmu/in/vehicle_visual_odometry"
    qos_depth: int = 10
    input_is_enu: bool = True
    offset_translation: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    offset_rpy_deg: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    quality: int = 100
    reset_counter: int = 0

    def normalized(self) -> "OdomBridgeConfig":
        """返回归一化后的配置副本。"""
        translation = _to_vec3(self.offset_translation, "offset_translation")
        rpy = _to_vec3(self.offset_rpy_deg, "offset_rpy_deg")
        return OdomBridgeConfig(
            input_topic=str(self.input_topic),
            output_topic=str(self.output_topic),
            qos_depth=max(1, int(self.qos_depth)),
            input_is_enu=bool(self.input_is_enu),
            offset_translation=translation,
            offset_rpy_deg=rpy,
            quality=max(-1, min(100, int(self.quality))),
            reset_counter=max(0, min(255, int(self.reset_counter))),
        )


def _to_vec3(value: Sequence[float], name: str) -> list[float]:
    """将输入规范化为 3 维浮点数组。"""
    if len(value) != 3:
        raise ValueError(f"{name} 需为长度为 3 的数组")
    return [float(value[0]), float(value[1]), float(value[2])]


def _mat_mul(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    """3x3 矩阵乘法。"""
    out: list[list[float]] = [[0.0, 0.0, 0.0] for _ in range(3)]
    for i in range(3):
        for j in range(3):
            out[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j]
    return out


def _mat_vec_mul(m: list[list[float]], v: Sequence[float]) -> list[float]:
    """3x3 矩阵与 3 维向量乘法。"""
    return [
        m[0][0] * float(v[0]) + m[0][1] * float(v[1]) + m[0][2] * float(v[2]),
        m[1][0] * float(v[0]) + m[1][1] * float(v[1]) + m[1][2] * float(v[2]),
        m[2][0] * float(v[0]) + m[2][1] * float(v[1]) + m[2][2] * float(v[2]),
    ]


def _quat_to_rot(qw: float, qx: float, qy: float, qz: float) -> list[list[float]]:
    """四元数(wxyz)转旋转矩阵。"""
    norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if norm < 1e-12:
        return [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ]
    qw, qx, qy, qz = qw / norm, qx / norm, qy / norm, qz / norm
    return [
        [
            1.0 - 2.0 * (qy * qy + qz * qz),
            2.0 * (qx * qy - qz * qw),
            2.0 * (qx * qz + qy * qw),
        ],
        [
            2.0 * (qx * qy + qz * qw),
            1.0 - 2.0 * (qx * qx + qz * qz),
            2.0 * (qy * qz - qx * qw),
        ],
        [
            2.0 * (qx * qz - qy * qw),
            2.0 * (qy * qz + qx * qw),
            1.0 - 2.0 * (qx * qx + qy * qy),
        ],
    ]


def _rot_to_quat(r: list[list[float]]) -> list[float]:
    """旋转矩阵转四元数(wxyz)。"""
    trace = r[0][0] + r[1][1] + r[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (r[2][1] - r[1][2]) / s
        qy = (r[0][2] - r[2][0]) / s
        qz = (r[1][0] - r[0][1]) / s
    elif r[0][0] > r[1][1] and r[0][0] > r[2][2]:
        s = math.sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]) * 2.0
        qw = (r[2][1] - r[1][2]) / s
        qx = 0.25 * s
        qy = (r[0][1] + r[1][0]) / s
        qz = (r[0][2] + r[2][0]) / s
    elif r[1][1] > r[2][2]:
        s = math.sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]) * 2.0
        qw = (r[0][2] - r[2][0]) / s
        qx = (r[0][1] + r[1][0]) / s
        qy = 0.25 * s
        qz = (r[1][2] + r[2][1]) / s
    else:
        s = math.sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]) * 2.0
        qw = (r[1][0] - r[0][1]) / s
        qx = (r[0][2] + r[2][0]) / s
        qy = (r[1][2] + r[2][1]) / s
        qz = 0.25 * s
    norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if norm < 1e-12:
        return [1.0, 0.0, 0.0, 0.0]
    return [qw / norm, qx / norm, qy / norm, qz / norm]

def euler_from_quat(qw: float, qx: float, qy: float, qz: float) -> tuple[float, float, float]:
    """从四元数计算欧拉角。"""
    yaw = math.atan2(2.0 * (qx * qy + qw * qz), qw * qw + qx * qx - qy * qy - qz * qz)
    pitch = math.asin(-2.0 * (qx * qz - qw * qy))
    roll = math.atan2(2.0 * (qy * qz + qw * qx), qw * qw - qx * qx - qy * qy + qz * qz)
    return roll, pitch, yaw


def _rpy_deg_to_rot(roll_deg: float, pitch_deg: float, yaw_deg: float) -> list[list[float]]:
    """欧拉角(度)按 ZYX 顺序转旋转矩阵。"""
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


class OdomToVehicleOdometryBridgeNode(Node):
    """订阅 nav_msgs/Odometry，发布 px4_msgs/VehicleOdometry。"""

    _ENU_TO_NED = [
        [0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
    ]
    _FRD_TO_FLU = [
        [1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
        [0.0, 0.0, -1.0],
    ]

    def __init__(self, config: OdomBridgeConfig | None = None) -> None:
        super().__init__("odom_to_vehicle_odometry_bridge")
        cfg = (config or OdomBridgeConfig()).normalized()
        self._input_is_enu = cfg.input_is_enu
        self._offset_translation = cfg.offset_translation
        offset_rpy = cfg.offset_rpy_deg
        self._offset_rot = _rpy_deg_to_rot(offset_rpy[0], offset_rpy[1], offset_rpy[2])
        self._quality = cfg.quality
        self._reset_counter = cfg.reset_counter

        qos = QoSProfile(depth=cfg.qos_depth)
        self._pub = self.create_publisher(VehicleOdometry, cfg.output_topic, qos)
        self._sub = self.create_subscription(Odometry, cfg.input_topic, self._on_odom, qos)

        # self.get_logger().info(
        #     "启动 odom->mocap_odometry 桥接: %s -> %s, input_is_enu=%s, offset_t=%s, offset_rpy_deg=%s",
        #     cfg.input_topic,
        #     cfg.output_topic,
        #     self._input_is_enu,
        #     [round(x, 6) for x in self._offset_translation],
        #     [round(x, 6) for x in offset_rpy],
        # )

    def _stamp_us(self, msg: Odometry) -> int:
        stamp = msg.header.stamp
        if int(stamp.sec) == 0 and int(stamp.nanosec) == 0:
            return int(time.time() * 1_000_000)
        return int(stamp.sec) * 1_000_000 + int(stamp.nanosec) // 1000

    def _on_odom(self, msg: Odometry) -> None:
        pos = [
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            float(msg.pose.pose.position.z),
        ]
        q_ros = msg.pose.pose.orientation
        # geometry_msgs/Quaternion 顺序为 xyzw，这里统一转成 wxyz。
        qw, qx, qy, qz = float(q_ros.w), float(q_ros.x), float(q_ros.y), float(q_ros.z)
        rot_odom_body = _quat_to_rot(qw, qx, qy, qz)

        v_child = [
            float(msg.twist.twist.linear.x),
            float(msg.twist.twist.linear.y),
            float(msg.twist.twist.linear.z),
        ]
        w_child = [
            float(msg.twist.twist.angular.x),
            float(msg.twist.twist.angular.y),
            float(msg.twist.twist.angular.z),
        ]

        if self._input_is_enu:
            pos_out = _mat_vec_mul(self._ENU_TO_NED, pos)
            rot_ned_frd = _mat_mul(
                _mat_mul(self._ENU_TO_NED, rot_odom_body),
                self._FRD_TO_FLU,
            )
            v_ned = _mat_vec_mul(self._ENU_TO_NED, _mat_vec_mul(rot_odom_body, v_child))
            w_frd = _mat_vec_mul(self._FRD_TO_FLU, w_child)
        else:
            pos_out = pos
            rot_ned_frd = rot_odom_body
            v_ned = _mat_vec_mul(rot_odom_body, v_child)
            w_frd = w_child

        # 应用初始偏移：p' = R_offset * p + t, R' = R_offset * R。
        pos_out = _mat_vec_mul(self._offset_rot, pos_out)
        pos_out = [
            pos_out[0] + self._offset_translation[0],
            pos_out[1] + self._offset_translation[1],
            pos_out[2] + self._offset_translation[2],
        ]
        rot_ned_frd = _mat_mul(self._offset_rot, rot_ned_frd)
        v_ned = _mat_vec_mul(self._offset_rot, v_ned)

        q_out = _rot_to_quat(rot_ned_frd)
        # 转欧拉角显示
        # roll, pitch, yaw = euler_from_quat(q_out[0], q_out[1], q_out[2], q_out[3])
        # print(f"roll: {math.degrees(roll)}, pitch: {math.degrees(pitch)}, yaw: {math.degrees(yaw)}")

        out = VehicleOdometry()
        out.timestamp = self._stamp_us(msg)
        out.timestamp_sample = out.timestamp
        out.pose_frame = VehicleOdometry.POSE_FRAME_FRD
        out.position = [float(pos_out[0]), float(pos_out[1]), float(pos_out[2])]
        out.q = [float(q_out[0]), float(q_out[1]), float(q_out[2]), float(q_out[3])]
        out.velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN
        out.velocity = [float(math.nan), float(math.nan), float(math.nan)]
        out.angular_velocity = [float(math.nan), float(math.nan), float(math.nan)]
        out.position_variance = [
            float(msg.pose.covariance[0]),
            float(msg.pose.covariance[7]),
            float(msg.pose.covariance[14]),
        ]
        out.orientation_variance = [
            float(msg.pose.covariance[21]),
            float(msg.pose.covariance[28]),
            float(msg.pose.covariance[35]),
        ]
        out.velocity_variance = [
            float(msg.twist.covariance[0]),
            float(msg.twist.covariance[7]),
            float(msg.twist.covariance[14]),
        ]
        out.reset_counter = max(0, min(255, self._reset_counter))
        out.quality = max(-1, min(100, self._quality))
        self._pub.publish(out)


def run_with_config(config: OdomBridgeConfig | None = None) -> int:
    """节点入口。"""
    if not ROS2_AVAILABLE:
        raise RuntimeError("未检测到 ROS2 环境，请先 source ROS2 与 px4_msgs")
    rclpy.init(args=None)
    node = OdomToVehicleOdometryBridgeNode(config=config)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def main() -> int:
    """默认配置启动入口。"""
    return run_with_config()


if __name__ == "__main__":
    # raise SystemExit(main())
    main()

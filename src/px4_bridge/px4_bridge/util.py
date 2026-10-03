import math

import numpy as np

# 世界系 ENU ↔ NED：交换 X/Y，翻转 Z
_R_WORLD = np.array(
    [
        [0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
    ]
)
# 机体系 FLU ↔ FRD：翻转 Y/Z（PX4 用 FRD，ROS/ENU 常用 FLU）
_R_BODY = np.array(
    [
        [1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
        [0.0, 0.0, -1.0],
    ]
)

_IDENTITY_QUAT = (1.0, 0.0, 0.0, 0.0)


def _quat_wxyz_to_rot(quat_wxyz):
    """[qw, qx, qy, qz] → 旋转矩阵。"""
    qw, qx, qy, qz = [float(v) for v in quat_wxyz]
    n = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if n < 1e-12:
        return np.eye(3)
    qw, qx, qy, qz = qw / n, qx / n, qy / n, qz / n
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ]
    )


def _rot_to_quat_wxyz(rot):
    """旋转矩阵 → [qw, qx, qy, qz]。"""
    m = np.asarray(rot, dtype=float)
    t = float(np.trace(m))
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m[2, 1] - m[1, 2]) / s
        qy = (m[0, 2] - m[2, 0]) / s
        qz = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        qw = (m[2, 1] - m[1, 2]) / s
        qx = 0.25 * s
        qy = (m[0, 1] + m[1, 0]) / s
        qz = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        qw = (m[0, 2] - m[2, 0]) / s
        qx = (m[0, 1] + m[1, 0]) / s
        qy = 0.25 * s
        qz = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        qw = (m[1, 0] - m[0, 1]) / s
        qx = (m[0, 2] + m[2, 0]) / s
        qy = (m[1, 2] + m[2, 1]) / s
        qz = 0.25 * s
    return [float(qw), float(qx), float(qy), float(qz)]


def _convert_attitude(quat_in):
    """
    ENU/FLU ↔ NED/FRD 姿态转换（双向同一公式）。

    R_out = R_world @ R_in @ R_body

    这样 ENU 单位姿态（机头朝东/X）对应 NED 下机头朝东（Y，yaw=+90°）。
    """
    rot_in = _quat_wxyz_to_rot(quat_in)
    rot_out = _R_WORLD @ rot_in @ _R_BODY
    return _rot_to_quat_wxyz(rot_out)


def transform_world_vec(vec):
    """世界系向量 ENU ↔ NED（位置/速度），双向同一变换。"""
    out = _R_WORLD @ np.asarray(vec, dtype=float)
    return (float(out[0]), float(out[1]), float(out[2]))


def transform_body_vec(vec):
    """机体系向量 FLU ↔ FRD（推力/角速度等），双向同一变换。"""
    out = _R_BODY @ np.asarray(vec, dtype=float)
    return (float(out[0]), float(out[1]), float(out[2]))


def convert_yaw(yaw: float) -> float:
    """ENU ↔ NED 偏航互转：out = -in + π/2（归一化到 [-π, π]）。"""
    return math.atan2(math.sin(-yaw + math.pi / 2), math.cos(-yaw + math.pi / 2))


def ned_to_enu(pos_ned, quat_ned=_IDENTITY_QUAT):
    """
    NED → ENU 转换（四元数输入）

    参数:
        pos_ned: [x, y, z] 在NED下的位置
        quat_ned: [qw, qx, qy, qz] 在NED/FRD下的姿态四元数（标量在前）

    返回:
        pos_enu: [x, y, z] 在ENU下的位置
        quat_enu: [qw, qx, qy, qz] 在ENU/FLU下的姿态四元数（标量在前）
    """
    pos_enu = transform_world_vec(pos_ned)
    quat_enu = _convert_attitude(quat_ned)
    return pos_enu, quat_enu


def enu_to_ned(pos_enu, quat_enu=_IDENTITY_QUAT):
    """
    ENU → NED 转换（四元数输入）

    参数:
        pos_enu: [x, y, z] 在ENU下的位置
        quat_enu: [qw, qx, qy, qz] 在ENU/FLU下的姿态四元数（标量在前）

    返回:
        pos_ned: [x, y, z] 在NED下的位置
        quat_ned: [qw, qx, qy, qz] 在NED/FRD下的姿态四元数（标量在前）
    """
    pos_ned = transform_world_vec(pos_enu)
    quat_ned = _convert_attitude(quat_enu)
    return pos_ned, quat_ned


def quat_to_euler(quat, degrees=False):
    """
    quat: [qw, qx, qy, qz]
    return: [roll, pitch, yaw]（xyz 内旋顺序）
    """
    qw, qx, qy, qz = [float(v) for v in quat]
    n = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if n < 1e-12:
        roll = pitch = yaw = 0.0
    else:
        qw, qx, qy, qz = qw / n, qx / n, qy / n, qz / n
        sinp = 2.0 * (qw * qy - qz * qx)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)
        roll = math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    if degrees:
        return np.degrees([roll, pitch, yaw])
    return np.array([roll, pitch, yaw], dtype=float)


if __name__ == "__main__":
    # test 1: ENU 单位姿态（机头朝东/X）→ NED 应机头朝东（Y），yaw≈+90°
    enu_pos = [1, 0, 0]
    enu_quat = [1, 0, 0, 0]
    ned_pos, ned_quat = enu_to_ned(enu_pos, enu_quat)
    print("ENU->[1,0,0] I -> NED pos/euler(roll,pitch,yaw):")
    print(ned_pos)
    print(quat_to_euler(ned_quat, degrees=True))
    print("yaw convert 0 ->", math.degrees(convert_yaw(0.0)))

    # test 2: NED 单位姿态（机头朝北/X）→ ENU 应机头朝北（Y），yaw≈+90°
    ned_pos = [1, 0, 0]
    ned_quat = [1, 0, 0, 0]
    enu_pos, enu_quat = ned_to_enu(ned_pos, ned_quat)
    print("NED I -> ENU pos/euler(roll,pitch,yaw):")
    print(enu_pos)
    print(quat_to_euler(enu_quat, degrees=True))

    # round-trip
    back_pos, back_quat = enu_to_ned(*ned_to_enu([1, 2, 3], [1, 0, 0, 0]))
    print("round-trip pos:", back_pos)
    print("round-trip quat:", np.round(back_quat, 6))
    print("thrust FLU +0.5 -> FRD", transform_body_vec([0.0, 0.0, 0.5]))

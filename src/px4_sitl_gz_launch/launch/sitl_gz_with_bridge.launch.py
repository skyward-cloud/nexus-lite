#!/usr/bin/env python3
#
# SPDX-License-Identifier: BSD-3-Clause
#
# 同时启动 PX4 SITL + gz (sitl_gz.launch.py) 与包内 gz_transport_listener。
#
#   ros2 launch px4_sitl_gz_launch sitl_gz_with_bridge.launch.py \\
#     px4_src:=/path/to/PX4-Autopilot
#
# use_sim_time 在本文件中固定为 false（SetParameter）。

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def _pkg_sim_paths():
    """包内仿真资源路径 (colcon install 后的 share/px4_sitl_gz_launch/{models,worlds})。"""
    pkg_share = get_package_share_directory('px4_sitl_gz_launch')
    return (
        os.path.join(pkg_share, 'worlds'),
        os.path.join(pkg_share, 'models'),
    )


def _sitl_launch_arguments():
    """与 sitl_gz.launch.py 中 DeclareLaunchArgument 保持一致，便于顶层 ros2 launch 传参。"""
    default_worlds_dir, default_models_dir = _pkg_sim_paths()
    return [
        DeclareLaunchArgument(
            'px4_src',
            default_value=os.environ.get('PX4_SOURCE_DIR', ''),
            description='PX4 源码根目录 (包含 build/ 与 Tools/simulation/gz)。可用环境变量 PX4_SOURCE_DIR。',
        ),
        DeclareLaunchArgument(
            'sitl_build',
            default_value='px4_sitl_default',
            description='构建目录名: build/<sitl_build>/bin/px4',
        ),
        DeclareLaunchArgument(
            'px4_sim_model',
            default_value='drone260',
            description='仿真机型: PX4_SIM_MODEL=gz_<名>, 对应 gz_bridge 加载 <名>/model.sdf; '
            '须在 GZ_SIM_RESOURCE_PATH 内可解析; 自定义模型见 sitl_gz.launch.py 头注释.',
        ),
        DeclareLaunchArgument(
            'px4_gz_world',
            default_value='custom',
            description='世界 basename (无 .sdf); 必须与 SDF 内 <world name="…"> 一致。',
        ),
        DeclareLaunchArgument(
            'px4_gz_worlds_dir',
            default_value=default_worlds_dir,
            description='覆盖 worlds 目录; 默认包内 share/.../worlds; 空则 <px4_src>/Tools/simulation/gz/worlds。',
        ),
        DeclareLaunchArgument(
            'gz_standalone',
            default_value='false',
            description='true: 不启动 gz (仅 PX4)。若 spawn_gz_first:=true, 则不再拉起 gz sim。',
        ),
        DeclareLaunchArgument(
            'spawn_gz_first',
            default_value='true',
            description='true: 在本 launch 内先 gz sim -s, 再延迟启动 px4。',
        ),
        DeclareLaunchArgument(
            'gz_ready_delay_sec',
            default_value='5.0',
            description='spawn_gz_first 时, gz sim 启动后等待多少秒再启动 px4。',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='true: 不启动 gz GUI (gz sim -g)。',
        ),
        DeclareLaunchArgument(
            'extra_gz_resource_path',
            default_value=default_models_dir,
            description='前置追加到 GZ_SIM_RESOURCE_PATH; 默认包内 share/.../models。',
        )
    ]


_SITL_ARG_NAMES = [
    'px4_src',
    'sitl_build',
    'px4_sim_model',
    'px4_gz_world',
    'px4_gz_worlds_dir',
    'gz_standalone',
    'spawn_gz_first',
    'gz_ready_delay_sec',
    'headless',
    'extra_gz_resource_path',
]


def _gz_listener_setup(context, *_args, **_kwargs):
    gz_imu_topic = LaunchConfiguration('gz_imu_topic').perform(context).strip()
    if not gz_imu_topic:
        gz_imu_topic = '/world/room/model/drone260_0/link/base_link/sensor/imu_sensor/imu'
    ros_imu_topic = LaunchConfiguration('ros_imu_topic').perform(context).strip() or '/imu/data'
    gz_pose_topic = LaunchConfiguration('gz_pose_topic').perform(context).strip() or '/model/drone260_0/pose'
    pose_entity_name = LaunchConfiguration('pose_entity_name').perform(context).strip() or 'drone260_0'
    ros_odom_topic = LaunchConfiguration('ros_odom_topic').perform(context).strip() or '/drone260/odom'
    gz_lidar_topic = LaunchConfiguration('gz_lidar_topic').perform(context).strip() or '/scan/points'
    ros_lidar_body_topic = LaunchConfiguration('ros_lidar_body_topic').perform(context).strip() or '/lidar/points_body'
    ros_lidar_world_topic = LaunchConfiguration('ros_lidar_world_topic').perform(context).strip() or '/lidar/points_world'

    raw_delay = LaunchConfiguration('gz_listener_delay_sec').perform(context).strip()
    try:
        delay_sec = float(raw_delay) if raw_delay else (
            float(LaunchConfiguration('gz_ready_delay_sec').perform(context).strip() or '5.0') + 1.0
        )
    except ValueError:
        delay_sec = 6.0

    node = Node(
        package='px4_sitl_gz_launch',
        executable='gz_transport_listener',
        arguments=[
            gz_imu_topic,
            ros_imu_topic,
            gz_pose_topic,
            pose_entity_name,
            ros_odom_topic,
            gz_lidar_topic,
            ros_lidar_body_topic,
            ros_lidar_world_topic,
        ],
        output='screen',
    )
    return [
        LogInfo(
            msg='[px4_sitl_gz_launch] starting gz_transport_listener '
            f'after {delay_sec}s '
            f'(imu: {gz_imu_topic}, pose: {gz_pose_topic}, entity: {pose_entity_name}, lidar: {gz_lidar_topic})'
        ),
        TimerAction(period=delay_sec, actions=[node]),
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('px4_sitl_gz_launch')
    sitl_launch = os.path.join(pkg_share, 'launch', 'sitl_gz.launch.py')

    forward = {name: LaunchConfiguration(name) for name in _SITL_ARG_NAMES}

    return LaunchDescription(
        _sitl_launch_arguments()
        + [
            # 全局关闭仿真时钟：本 launch 内 ROS 节点使用墙钟；后续自行起的节点也可继承该默认。
            SetParameter(name='use_sim_time', value=False),
            DeclareLaunchArgument(
                'gz_imu_topic',
                default_value='/world/room/model/drone260_0/link/base_link/sensor/imu_sensor/imu',
                description='gz_transport_listener 订阅的 Gazebo IMU 话题。',
            ),
            DeclareLaunchArgument(
                'ros_imu_topic',
                default_value='/imu/data',
                description='gz_transport_listener 发布的 ROS IMU 话题。',
            ),
            DeclareLaunchArgument(
                'gz_pose_topic',
                default_value='/model/drone260_0/pose',
                description='gz_transport_listener 订阅的 Gazebo Pose_V 话题。',
            ),
            DeclareLaunchArgument(
                'pose_entity_name',
                default_value='drone260_0',
                description='在 Pose_V 中用于匹配位姿的实体名（通常是模型名）。',
            ),
            DeclareLaunchArgument(
                'ros_odom_topic',
                default_value='/drone260/odom',
                description='gz_transport_listener 发布的 ROS 里程计话题。',
            ),
            DeclareLaunchArgument(
                'gz_lidar_topic',
                default_value='/scan/points',
                description='gz_transport_listener 订阅的 Gazebo 点云话题(PointCloudPacked)。',
            ),
            DeclareLaunchArgument(
                'ros_lidar_body_topic',
                default_value='/lidar/points_body',
                description='机体系(base_link) ROS 点云话题。',
            ),
            DeclareLaunchArgument(
                'ros_lidar_world_topic',
                default_value='/lidar/points_world',
                description='世界系(world) ROS 点云话题（由机体系点云+odom变换得到）。',
            ),
            DeclareLaunchArgument(
                'gz_listener_delay_sec',
                default_value='',
                description='gz_transport_listener 延迟启动秒数；空则使用 gz_ready_delay_sec+1 秒。',
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sitl_launch),
                launch_arguments=forward.items(),
            ),
            OpaqueFunction(function=_gz_listener_setup),
        ]
    )

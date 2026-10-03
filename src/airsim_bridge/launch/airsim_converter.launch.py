from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # 声明启动参数
        DeclareLaunchArgument(
            'ned_odom_topic',
            default_value='/airsim_node/PX4/odom_local_ned',
            description='Input NED odometry topic'
        ),
        DeclareLaunchArgument(
            'ned_pointcloud_topic',
            default_value='/airsim_node/PX4/lidar/LidarSensor1',
            description='Input NED pointcloud topic'
        ),
        DeclareLaunchArgument(
            'enu_odom_topic',
            default_value='/enu/odom',
            description='Output ENU odometry topic'
        ),
        DeclareLaunchArgument(
            'global_pointcloud_topic',
            default_value='/global/pointcloud',
            description='Output global pointcloud topic'
        ),
        DeclareLaunchArgument(
            'global_frame',
            default_value='world',
            description='Global frame name'
        ),
        DeclareLaunchArgument(
            'sensor_frame',
            default_value='sensor_link',
            description='Sensor frame name'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        # 启动NED到ENU转换节点
        Node(
            package='airsim_bridge',
            executable='transform',
            name='transform',
            output='screen',
            parameters=[{
                'ned_odom_topic': LaunchConfiguration('ned_odom_topic'),
                'ned_pointcloud_topic': LaunchConfiguration('ned_pointcloud_topic'),
                'enu_odom_topic': LaunchConfiguration('enu_odom_topic'),
                'global_pointcloud_topic': LaunchConfiguration('global_pointcloud_topic'),
                'global_frame': LaunchConfiguration('global_frame'),
                'sensor_frame': LaunchConfiguration('sensor_frame'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            remappings=[
                ('/ned/odom', LaunchConfiguration('ned_odom_topic')),
                ('/ned/pointcloud', LaunchConfiguration('ned_pointcloud_topic')),
                ('/enu/odom', LaunchConfiguration('enu_odom_topic')),
                ('/global/pointcloud', LaunchConfiguration('global_pointcloud_topic')),
            ]
        ),
    ])
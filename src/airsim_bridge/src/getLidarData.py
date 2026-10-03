import airsim


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
import numpy as np
# from scipy.spatial.transform import Rotation
import struct
import time


class PointCloudPublisher(Node):
    def __init__(self):
        super().__init__("pointcloud_publisher")

        # connect to the AirSim simulator
        self.client = airsim.MultirotorClient("192.168.138.1")
        self.client.confirmConnection()

        self.odom_translation = None
        self.odom_rotation_matrix = None

        # 创建PointCloud2发布器
        self.local_pointcloud_pub = self.create_publisher(
            PointCloud2, "/lidar/points_body", 10
        )
        self.global_pointcloud_pub = self.create_publisher(
            PointCloud2, "/lidar/points_world", 10
        )
        self.odom_pub = self.create_publisher(Odometry, "/drone260/odom", 10)

        # 创建定时器，以10Hz的频率发布点云
        self.timer = self.create_timer(0.05, self.publish_pointcloud)
        self.timer = self.create_timer(0.02, self.publish_odom)

        # 定义点云字段
        self.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

    def quaternion_to_rotation_matrix(self, x, y, z, w):
        """将四元数转换为旋转矩阵"""
        # 计算旋转矩阵的元素
        xx = x * x
        yy = y * y
        zz = z * z
        xy = x * y
        xz = x * z
        yz = y * z
        wx = w * x
        wy = w * y
        wz = w * z

        rotation_matrix = np.array(
            [
                [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
                [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
                [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
            ]
        )

        return rotation_matrix

    def transform_points_vectorized(self, points_array):
        """使用numpy向量化操作批量转换点云"""
        # points_array形状: (N, 3)
        # 应用旋转: (3,3) @ (N,3).T = (3, N) -> 转置后得到 (N, 3)
        rotated_points = (self.odom_rotation_matrix @ points_array.T).T
        # 应用平移
        transformed_points = rotated_points + self.odom_translation
        return transformed_points

    def publish_odom(self):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "world"
        odom_msg.child_frame_id = "base_link"
        try:
            pose = self.client.simGetVehiclePose()
            position = [pose.position.x_val, -pose.position.y_val, -pose.position.z_val]
            odom_msg.pose.pose.position.x = position[0]
            odom_msg.pose.pose.position.y = position[1]
            odom_msg.pose.pose.position.z = position[2]
            orientation = [
                pose.orientation.x_val,
                -pose.orientation.y_val,
                -pose.orientation.z_val,
                pose.orientation.w_val,
            ]
            # 绕x轴旋转180度
            odom_msg.pose.pose.orientation.x = orientation[0]
            odom_msg.pose.pose.orientation.y = orientation[1]
            odom_msg.pose.pose.orientation.z = orientation[2]
            odom_msg.pose.pose.orientation.w = orientation[3]
            self.odom_pub.publish(odom_msg)
            self.odom_rotation_matrix = self.quaternion_to_rotation_matrix(
                orientation[0], orientation[1], orientation[2], orientation[3]
            )
            self.odom_translation = np.array([position[0], position[1], position[2]])
        except Exception as e:
            self.get_logger().error(f"Failed to get pose: {e}")

    def publish_pointcloud(self):
        # 创建点云消息
        start_time = time.time()
        local_points = PointCloud2()
        global_points = PointCloud2()
        data = self.client.getLidarData("LidarSensor1")
        if len(data.point_cloud) < 3:
            return
        local_points.header.stamp = self.get_clock().now().to_msg()
        local_points.header.frame_id = "map"

        # 设置点云参数
        local_points.height = 1  # 无序点云
        local_points.width = len(data.point_cloud) // 3  # 点云点的数量

        # 设置点云字段
        local_points.fields = self.fields
        local_points.is_bigendian = False
        local_points.point_step = 12
        local_points.row_step = local_points.point_step * local_points.width
        local_points.is_dense = True

        # 设置点云参数
        global_points.header.stamp = local_points.header.stamp
        global_points.header.frame_id = "world"
        global_points.height = local_points.height
        global_points.width = local_points.width
        # 设置点云字段
        global_points.fields = local_points.fields
        global_points.is_bigendian = local_points.is_bigendian
        global_points.point_step = local_points.point_step
        global_points.row_step = local_points.row_step
        global_points.is_dense = local_points.is_dense

        # 交换xy轴并反转z轴
        # for i in range(0, len(data.point_cloud), 3):
        #     x, y, z = data.point_cloud[i:i+3]
        #     data.point_cloud[i:i+3] = [y, x, -z]

        # 将点云数据转换为numpy数组
        body_points = np.array(data.point_cloud).reshape(-1, 3)
        # 向量化交换和取反操作
        # body_points[:, [0, 1]] = body_points[:, [1, 0]]  # 交换x和y
        body_points[:, 2] = -body_points[:, 2]  # z取反
        body_points[:, 1] = -body_points[:, 1]  # y取反

        # 将结果重新赋值回原数据
        data.point_cloud = body_points.flatten().tolist()
        format_str = f"{len(data.point_cloud)}f"  # 例如 "10f" 表示10个float
        local_points.data = struct.pack(format_str, *data.point_cloud)
        self.local_pointcloud_pub.publish(local_points)

        # 全局点云 = body_points + latest_pose
        if self.odom_rotation_matrix is not None and self.odom_translation is not None:
            transformed_points = self.transform_points_vectorized(body_points)
            world_points = transformed_points.flatten().tolist()
            global_points.data = struct.pack(format_str, *world_points)
            self.global_pointcloud_pub.publish(global_points)
        # print(time.time() - start_time)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

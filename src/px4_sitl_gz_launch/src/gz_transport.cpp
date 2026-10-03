#include <gz/msgs/imu.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/transport/Node.hh>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace
{
struct PointXYZ
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  uint16_t ring{0U};
  float intensity{0.0F};
};

struct PoseState
{
  bool valid{false};
  double tx{0.0};
  double ty{0.0};
  double tz{0.0};
  double qx{0.0};
  double qy{0.0};
  double qz{0.0};
  double qw{1.0};
};

// rclcpp::Time ToRosStamp(const gz::msgs::Header &header, const rclcpp::Clock::SharedPtr &clock)
// {
//   if (header.has_stamp())
//   {
//     return rclcpp::Time(
//       static_cast<int32_t>(header.stamp().sec()),
//       static_cast<uint32_t>(header.stamp().nsec()),
//       RCL_ROS_TIME);
//   }
//   return clock->now();
// }

rclcpp::Time ToRosStamp(const gz::msgs::Header &header, const rclcpp::Clock::SharedPtr &clock)
{
  if (header.has_stamp())
  {
    // 获取系统当前时间
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
    auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration - seconds);
    
    return rclcpp::Time(
      static_cast<int32_t>(seconds.count()),
      static_cast<uint32_t>(nanoseconds.count()),
      RCL_ROS_TIME);
  }
  return clock->now();
}

template<typename T>
T ReadScalar(const std::string &data, size_t offset, bool bigendian)
{
  T value{};
  uint8_t *dst = reinterpret_cast<uint8_t *>(&value);
  if (bigendian)
  {
    for (size_t i = 0; i < sizeof(T); ++i)
    {
      dst[i] = static_cast<uint8_t>(data[offset + sizeof(T) - 1U - i]);
    }
  } else
  {
    std::memcpy(dst, data.data() + offset, sizeof(T));
  }
  return value;
}

bool ReadFieldAsFloat(
  const std::string &data, size_t base_offset, bool bigendian,
  uint32_t datatype, float &out_value)
{
  switch (datatype)
  {
    case gz::msgs::PointCloudPacked::Field::INT8:
      out_value = static_cast<float>(ReadScalar<int8_t>(data, base_offset, bigendian));
      return true;
    case gz::msgs::PointCloudPacked::Field::UINT8:
      out_value = static_cast<float>(ReadScalar<uint8_t>(data, base_offset, bigendian));
      return true;
    case gz::msgs::PointCloudPacked::Field::INT16:
      out_value = static_cast<float>(ReadScalar<int16_t>(data, base_offset, bigendian));
      return true;
    case gz::msgs::PointCloudPacked::Field::UINT16:
      out_value = static_cast<float>(ReadScalar<uint16_t>(data, base_offset, bigendian));
      return true;
    case gz::msgs::PointCloudPacked::Field::INT32:
      out_value = static_cast<float>(ReadScalar<int32_t>(data, base_offset, bigendian));
      return true;
    case gz::msgs::PointCloudPacked::Field::UINT32:
      out_value = static_cast<float>(ReadScalar<uint32_t>(data, base_offset, bigendian));
      return true;
    case gz::msgs::PointCloudPacked::Field::FLOAT32:
      out_value = ReadScalar<float>(data, base_offset, bigendian);
      return true;
    case gz::msgs::PointCloudPacked::Field::FLOAT64:
      out_value = static_cast<float>(ReadScalar<double>(data, base_offset, bigendian));
      return true;
    default:
      return false;
  }
}

void RotatePointByQuaternion(
  const PoseState &pose, float x, float y, float z, float &rx, float &ry, float &rz)
{
  const double qx = pose.qx;
  const double qy = pose.qy;
  const double qz = pose.qz;
  const double qw = pose.qw;

  const double r00 = 1.0 - 2.0 * (qy * qy + qz * qz);
  const double r01 = 2.0 * (qx * qy - qz * qw);
  const double r02 = 2.0 * (qx * qz + qy * qw);
  const double r10 = 2.0 * (qx * qy + qz * qw);
  const double r11 = 1.0 - 2.0 * (qx * qx + qz * qz);
  const double r12 = 2.0 * (qy * qz - qx * qw);
  const double r20 = 2.0 * (qx * qz - qy * qw);
  const double r21 = 2.0 * (qy * qz + qx * qw);
  const double r22 = 1.0 - 2.0 * (qx * qx + qy * qy);

  rx = static_cast<float>(r00 * x + r01 * y + r02 * z + pose.tx);
  ry = static_cast<float>(r10 * x + r11 * y + r12 * z + pose.ty);
  rz = static_cast<float>(r20 * x + r21 * y + r22 * z + pose.tz);
}

sensor_msgs::msg::PointCloud2 BuildCloudMsg(
  const std::vector<PointXYZ> &points, const rclcpp::Time &stamp, const std::string &frame_id)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = stamp;
  cloud.header.frame_id = frame_id;
  cloud.height = 1;
  cloud.width = static_cast<uint32_t>(points.size());

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(
    5,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "ring", 1, sensor_msgs::msg::PointField::UINT16,
    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<uint16_t> iter_ring(cloud, "ring");
  sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud, "intensity");

  for (const auto &p : points)
  {
    *iter_x = p.x;
    *iter_y = p.y;
    *iter_z = p.z;
    *iter_ring = p.ring;
    *iter_intensity = p.intensity;
    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_ring;
    ++iter_intensity;
  }
  return cloud;
}

bool DecodeXYZPoints(const gz::msgs::PointCloudPacked &msg, std::vector<PointXYZ> &out_points)
{
  int32_t x_offset = -1;
  int32_t y_offset = -1;
  int32_t z_offset = -1;
  int32_t ring_offset = -1;
  int32_t intensity_offset = -1;
  uint32_t x_type = 0;
  uint32_t y_type = 0;
  uint32_t z_type = 0;
  uint32_t ring_type = 0;
  uint32_t intensity_type = 0;

  for (int i = 0; i < msg.field_size(); ++i)
  {
    const auto &field = msg.field(i);
    if (field.name() == "x")
    {
      x_offset = static_cast<int32_t>(field.offset());
      x_type = field.datatype();
    } else if (field.name() == "y")
    {
      y_offset = static_cast<int32_t>(field.offset());
      y_type = field.datatype();
    } else if (field.name() == "z")
    {
      z_offset = static_cast<int32_t>(field.offset());
      z_type = field.datatype();
    } else if (field.name() == "ring")
    {
      ring_offset = static_cast<int32_t>(field.offset());
      ring_type = field.datatype();
    } else if (field.name() == "intensity")
    {
      intensity_offset = static_cast<int32_t>(field.offset());
      intensity_type = field.datatype();
    }
  }

  if (x_offset < 0 || y_offset < 0 || z_offset < 0 || msg.point_step() == 0)
  {
    return false;
  }

  const uint32_t width = msg.width() > 0 ? msg.width() : static_cast<uint32_t>(msg.data().size() / msg.point_step());
  const uint32_t height = msg.height() > 0 ? msg.height() : 1U;
  const uint32_t row_step = msg.row_step() > 0 ? msg.row_step() : msg.point_step() * width;
  const bool bigendian = msg.is_bigendian();
  const std::string &raw = msg.data();

  out_points.clear();
  out_points.reserve(static_cast<size_t>(width) * static_cast<size_t>(height));

  for (uint32_t r = 0; r < height; ++r)
  {
    for (uint32_t c = 0; c < width; ++c)
    {
      const size_t point_base = static_cast<size_t>(r) * row_step + static_cast<size_t>(c) * msg.point_step();
      const size_t x_idx = point_base + static_cast<size_t>(x_offset);
      const size_t y_idx = point_base + static_cast<size_t>(y_offset);
      const size_t z_idx = point_base + static_cast<size_t>(z_offset);
      const size_t ring_idx = (ring_offset >= 0)
                                ? point_base + static_cast<size_t>(ring_offset)
                                : point_base;
      const size_t intensity_idx = (intensity_offset >= 0)
                                     ? point_base + static_cast<size_t>(intensity_offset)
                                     : point_base;
      if (x_idx >= raw.size() || y_idx >= raw.size() || z_idx >= raw.size())
      {
        return true;
      }

      float x = 0.0F;
      float y = 0.0F;
      float z = 0.0F;
      if (!ReadFieldAsFloat(raw, x_idx, bigendian, x_type, x) ||
        !ReadFieldAsFloat(raw, y_idx, bigendian, y_type, y) ||
        !ReadFieldAsFloat(raw, z_idx, bigendian, z_type, z))
      {
        return false;
      }

      if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z))
      {
        uint16_t ring = 0U;
        float intensity = 0.0F;
        if (ring_offset >= 0 && ring_idx < raw.size())
        {
          float ring_f = 0.0F;
          if (ReadFieldAsFloat(raw, ring_idx, bigendian, ring_type, ring_f) && std::isfinite(ring_f))
          {
            ring = static_cast<uint16_t>(std::clamp(ring_f, 0.0F, 65535.0F));
          }
        }
        if (intensity_offset >= 0 && intensity_idx < raw.size())
        {
          float intensity_f = 0.0F;
          if (ReadFieldAsFloat(raw, intensity_idx, bigendian, intensity_type, intensity_f) &&
            std::isfinite(intensity_f))
          {
            intensity = intensity_f;
          }
        }
        out_points.push_back(PointXYZ{x, y, z, ring, intensity});
      }
    }
  }
  return true;
}
}  // namespace

class GzBridgeNode : public rclcpp::Node
{
public:
  GzBridgeNode(
    const std::string &gz_imu_topic, const std::string &ros_imu_topic,
    const std::string &gz_pose_topic, const std::string &pose_entity_name, const std::string &ros_odom_topic,
    const std::string &gz_lidar_topic, const std::string &ros_lidar_topic,
    const std::string &ros_world_lidar_topic)
  : Node("gz_transport_listener"), pose_entity_name_(pose_entity_name)
  {
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(ros_imu_topic, 10);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(ros_odom_topic, 10);
    lidar_body_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(ros_lidar_topic, 10);
    lidar_world_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(ros_world_lidar_topic, 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    if (!gz_node_.Subscribe(gz_imu_topic, &GzBridgeNode::OnImu, this))
    {
      throw std::runtime_error("Failed to subscribe gz imu topic: " + gz_imu_topic);
    }
    if (!gz_node_.Subscribe(gz_pose_topic, &GzBridgeNode::OnPoseV, this))
    {
      throw std::runtime_error("Failed to subscribe gz pose topic: " + gz_pose_topic);
    }
    if (!gz_node_.Subscribe(gz_lidar_topic, &GzBridgeNode::OnLidar, this))
    {
      throw std::runtime_error("Failed to subscribe gz lidar topic: " + gz_lidar_topic);
    }

    RCLCPP_INFO(this->get_logger(), "Subscribed GZ IMU topic: %s", gz_imu_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Subscribed GZ POSE topic: %s", gz_pose_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Pose entity name: %s", pose_entity_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Subscribed GZ LIDAR topic: %s", gz_lidar_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing ROS IMU topic: %s", ros_imu_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing ROS ODOM topic: %s", ros_odom_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing TF: world -> base_link");
    RCLCPP_INFO(this->get_logger(), "Publishing ROS body cloud topic: %s", ros_lidar_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing ROS world cloud topic: %s", ros_world_lidar_topic.c_str());
  }

private:
  void OnImu(const gz::msgs::IMU &msg, const gz::transport::MessageInfo &info)
  {
    (void)info;
    sensor_msgs::msg::Imu out;
    out.header.stamp = ToRosStamp(msg.header(), this->get_clock());
    out.header.frame_id = "base_link";

    if (msg.has_orientation())
    {
      out.orientation.x = msg.orientation().x();
      out.orientation.y = msg.orientation().y();
      out.orientation.z = msg.orientation().z();
      out.orientation.w = msg.orientation().w();
    } else
    {
      out.orientation.w = 1.0;
    }

    out.angular_velocity.x = msg.angular_velocity().x();
    out.angular_velocity.y = msg.angular_velocity().y();
    out.angular_velocity.z = msg.angular_velocity().z();
    out.linear_acceleration.x = msg.linear_acceleration().x();
    out.linear_acceleration.y = msg.linear_acceleration().y();
    out.linear_acceleration.z = msg.linear_acceleration().z();
    imu_pub_->publish(out);
  }

  void OnPoseV(const gz::msgs::Pose_V &msg, const gz::transport::MessageInfo &info)
  {
    (void)info;
    const gz::msgs::Pose *matched = nullptr;
    for (int i = 0; i < msg.pose_size(); ++i)
    {
      const auto &p = msg.pose(i);
      if (p.name() == pose_entity_name_)
      {
        matched = &p;
        break;
      }
    }
    if (matched == nullptr)
    {
      return;
    }

    const rclcpp::Time stamp = ToRosStamp(msg.header(), this->get_clock());

    nav_msgs::msg::Odometry out;
    out.header.stamp = stamp;
    out.header.frame_id = "world";
    out.child_frame_id = "base_link";
    out.pose.pose.position.x = matched->position().x();
    out.pose.pose.position.y = matched->position().y();
    out.pose.pose.position.z = matched->position().z();
    out.pose.pose.orientation.x = matched->orientation().x();
    out.pose.pose.orientation.y = matched->orientation().y();
    out.pose.pose.orientation.z = matched->orientation().z();
    out.pose.pose.orientation.w = matched->orientation().w();
    odom_pub_->publish(out);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = "world";
    tf.child_frame_id = "base_link";
    tf.transform.translation.x = out.pose.pose.position.x;
    tf.transform.translation.y = out.pose.pose.position.y;
    tf.transform.translation.z = out.pose.pose.position.z;
    tf.transform.rotation = out.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);

    std::lock_guard<std::mutex> lk(pose_mutex_);
    latest_pose_.valid = true;
    latest_pose_.tx = out.pose.pose.position.x;
    latest_pose_.ty = out.pose.pose.position.y;
    latest_pose_.tz = out.pose.pose.position.z;
    latest_pose_.qx = out.pose.pose.orientation.x;
    latest_pose_.qy = out.pose.pose.orientation.y;
    latest_pose_.qz = out.pose.pose.orientation.z;
    latest_pose_.qw = out.pose.pose.orientation.w;
  }

  void OnLidar(const gz::msgs::PointCloudPacked &msg, const gz::transport::MessageInfo &info)
  {
    (void)info;
    std::vector<PointXYZ> body_points;
    if (!DecodeXYZPoints(msg, body_points))
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Failed to decode XYZ from PointCloudPacked");
      return;
    }

    const rclcpp::Time stamp = ToRosStamp(msg.header(), this->get_clock());
    lidar_body_pub_->publish(BuildCloudMsg(body_points, stamp, "base_link"));

    PoseState pose;
    {
      std::lock_guard<std::mutex> lk(pose_mutex_);
      pose = latest_pose_;
    }
    if (!pose.valid)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "No pose yet, skip world cloud");
      return;
    }

    std::vector<PointXYZ> world_points;
    world_points.reserve(body_points.size());
    for (const auto &p : body_points)
    {
      PointXYZ pw;
      RotatePointByQuaternion(pose, p.x, p.y, p.z, pw.x, pw.y, pw.z);
      pw.ring = p.ring;
      pw.intensity = p.intensity;
      world_points.push_back(pw);
    }
    lidar_world_pub_->publish(BuildCloudMsg(world_points, stamp, "world"));
  }

  gz::transport::Node gz_node_;
  std::string pose_entity_name_;
  std::mutex pose_mutex_;
  PoseState latest_pose_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_body_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_world_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  const auto non_ros_args = rclcpp::remove_ros_arguments(argc, argv);

  const std::string gz_imu_topic = (non_ros_args.size() > 1)
                                     ? non_ros_args[1]
                                     : "/world/room/model/drone260_0/link/base_link/sensor/imu_sensor/imu";
  const std::string ros_imu_topic = (non_ros_args.size() > 2) ? non_ros_args[2] : "/imu/data";
  const std::string gz_pose_topic = (non_ros_args.size() > 3) ? non_ros_args[3] : "/model/drone260_0/pose";
  const std::string pose_entity_name = (non_ros_args.size() > 4) ? non_ros_args[4] : "drone260_0";
  const std::string ros_odom_topic = (non_ros_args.size() > 5) ? non_ros_args[5] : "/drone260/odom";
  const std::string gz_lidar_topic = (non_ros_args.size() > 6) ? non_ros_args[6] : "/scan/points";
  const std::string ros_lidar_topic = (non_ros_args.size() > 7) ? non_ros_args[7] : "/lidar/points_body";
  const std::string ros_world_lidar_topic = (non_ros_args.size() > 8) ? non_ros_args[8] : "/lidar/points_world";

  try
  {
    auto node = std::make_shared<GzBridgeNode>(
      gz_imu_topic, ros_imu_topic, gz_pose_topic, pose_entity_name, ros_odom_topic,
      gz_lidar_topic, ros_lidar_topic, ros_world_lidar_topic);
    rclcpp::spin(node);
  } catch (const std::exception &e)
  {
    std::cerr << "[gz_transport_listener] " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
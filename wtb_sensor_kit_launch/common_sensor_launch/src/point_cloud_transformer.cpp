#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/convert.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "autoware/point_types/types.hpp"

#include <cstring>
#include <algorithm>

using PointXYZIRC = autoware::point_types::PointXYZIRC;

// PointXYZIRT: Velodyne-compatible point with ring and per-point time offset.
// LIO-SAM expects this exact field layout (x, y, z, intensity, ring, time).
struct PointXYZIRT
{
  PCL_ADD_POINT4D;
  PCL_ADD_INTENSITY;
  std::uint16_t ring;
  float time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, intensity, intensity)
  (std::uint16_t, ring, ring)
  (float, time, time)
)

namespace
{
int field_offset(const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name)
{
  for (const auto & field : cloud.fields) {
    if (field.name == name) {
      return static_cast<int>(field.offset);
    }
  }
  return -1;
}

int field_datatype(const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name)
{
  for (const auto & field : cloud.fields) {
    if (field.name == name) {
      return static_cast<int>(field.datatype);
    }
  }
  return -1;
}

uint16_t read_uint16(const uint8_t * point, int offset)
{
  if (offset < 0) return 0U;
  uint16_t value = 0U;
  std::memcpy(&value, point + offset, sizeof(uint16_t));
  return value;
}
}  // namespace

class PointCloudTransformer2 : public rclcpp::Node
{
public:
  explicit PointCloudTransformer2(const rclcpp::NodeOptions & options)
  : Node("point_cloud_transformer", options),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    this->declare_parameter<double>("min_height_threshold", -1.00);
    this->get_parameter("min_height_threshold", min_height_threshold);
    this->declare_parameter<double>("max_height_threshold", 1.00);
    this->get_parameter("max_height_threshold", max_height_threshold);

    this->declare_parameter<bool>("use_height_filter", false);
    this->get_parameter("use_height_filter", use_height_filter);

    point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/sensing/lidar/pointcloud_raw",
      rclcpp::QoS(5),
      std::bind(&PointCloudTransformer2::pointCloudCallback, this, std::placeholders::_1));

    point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/sensing/lidar/concatenated/pointcloud",
      rclcpp::QoS(5).reliable().durability_volatile());

    // Second publisher: PointXYZIRT format for LIO-SAM (preserves ring + time)
    point_cloud_pub_xyzirt_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/sensing/lidar/concatenated/pointcloud_xyzirt",
      rclcpp::QoS(5).reliable().durability_volatile());

    RCLCPP_INFO(this->get_logger(), "Waiting for transform from 'laser' to 'base_link'...");
    while (!tf_buffer_.canTransform("base_link", "laser", rclcpp::Time())) {
      if (!rclcpp::ok()) return;
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    try {
      transform_ = tf_buffer_.lookupTransform("base_link", "laser", rclcpp::Time());
      RCLCPP_INFO(this->get_logger(), "TF transform acquired.");
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(this->get_logger(), "Failed to lookup transform: %s", ex.what());
    }
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // --- resolve field offsets once ---
    if (ring_offset_ < 0) {
      ring_offset_ = field_offset(*msg, "ring");
      if (ring_offset_ < 0) ring_offset_ = field_offset(*msg, "channel");
    }
    // Time: RoboSense uses "timestamp" (double, 8 bytes), Velodyne uses "time" (float, 4 bytes)
    if (time_offset_ < 0) {
      time_offset_   = field_offset(*msg, "timestamp");
      time_datatype_ = field_datatype(*msg, "timestamp");
      if (time_offset_ < 0) {
        time_offset_   = field_offset(*msg, "time");
        time_datatype_ = field_datatype(*msg, "time");
      }
      if (time_offset_ < 0) {
        time_offset_   = field_offset(*msg, "t");
        time_datatype_ = field_datatype(*msg, "t");
      }
    }

    // Step 1: read input (xyz + intensity only)
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>);
    try {
      pcl::fromROSMsg(*msg, *cloud_in);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to convert input PointCloud2: %s", e.what());
      return;
    }

    // Step 2: two output clouds in a single pass
    //   cloud_autoware: PointXYZIRC (for Autoware NDT / perception / filters)
    //   cloud_liosam:   PointXYZIRT (for LIO-SAM, preserves ring + time)
    auto cloud_autoware = std::make_shared<pcl::PointCloud<PointXYZIRC>>();
    auto cloud_liosam   = std::make_shared<pcl::PointCloud<PointXYZIRT>>();
    cloud_autoware->height   = cloud_liosam->height   = 1;
    cloud_autoware->is_dense = cloud_liosam->is_dense = cloud_in->is_dense;
    cloud_autoware->points.reserve(cloud_in->points.size());
    cloud_liosam->points.reserve(cloud_in->points.size());

    // Step 3: TF transform matrix (laser → base_link)
    Eigen::Matrix4f T = tf2::transformToEigen(transform_.transform).matrix().cast<float>();

    // Step 4: per-point transform — build both clouds in one loop
    const auto * raw_data   = msg->data.data();
    const int    point_step = static_cast<int>(msg->point_step);
    const double header_stamp_sec = rclcpp::Time(msg->header.stamp).seconds();
    const bool   has_time   = (time_offset_ >= 0);

    for (size_t i = 0; i < cloud_in->points.size(); ++i) {
      const auto & p_in = cloud_in->points[i];

      Eigen::Vector4f pt(p_in.x, p_in.y, p_in.z, 1.0f);
      Eigen::Vector4f pt_t = T * pt;

      if (use_height_filter) {
        if (pt_t.z() < min_height_threshold || pt_t.z() > max_height_threshold)
          continue;
      }

      const uint16_t ring_val = read_uint16(raw_data + i * point_step, ring_offset_);

      // --- Autoware format (PointXYZIRC) ---
      {
        PointXYZIRC p_aw;
        p_aw.x           = pt_t.x();
        p_aw.y           = pt_t.y();
        p_aw.z           = pt_t.z();
        p_aw.intensity   = static_cast<std::uint8_t>(
            std::min(255.0f, std::max(0.0f, p_in.intensity)));
        p_aw.return_type = static_cast<std::uint8_t>(
            autoware::point_types::ReturnType::SINGLE_STRONGEST);
        p_aw.channel     = ring_val;
        cloud_autoware->points.push_back(p_aw);
      }

      // --- LIO-SAM format (PointXYZIRT) ---
      {
        PointXYZIRT p_ls;
        p_ls.x         = pt_t.x();
        p_ls.y         = pt_t.y();
        p_ls.z         = pt_t.z();
        p_ls.intensity = p_in.intensity;
        p_ls.ring      = ring_val;

        if (has_time) {
          double abs_time = 0.0;
          const uint8_t * field_ptr = raw_data + i * point_step + time_offset_;
          if (time_datatype_ == sensor_msgs::msg::PointField::FLOAT64) {
            std::memcpy(&abs_time, field_ptr, sizeof(double));
          } else if (time_datatype_ == sensor_msgs::msg::PointField::FLOAT32) {
            float ft;
            std::memcpy(&ft, field_ptr, sizeof(float));
            abs_time = static_cast<double>(ft);
          } else if (time_datatype_ == sensor_msgs::msg::PointField::UINT32) {
            uint32_t ut;
            std::memcpy(&ut, field_ptr, sizeof(uint32_t));
            abs_time = static_cast<double>(ut) * 1e-9;
          }
          p_ls.time = static_cast<float>(abs_time - header_stamp_sec);
        } else {
          p_ls.time = 0.0f;
        }
        cloud_liosam->points.push_back(p_ls);
      }
    }

    cloud_autoware->width = cloud_autoware->points.size();
    cloud_liosam->width   = cloud_liosam->points.size();

    // Step 5: publish both
    {
      sensor_msgs::msg::PointCloud2 out_aw;
      pcl::toROSMsg(*cloud_autoware, out_aw);
      out_aw.header.stamp    = msg->header.stamp;
      out_aw.header.frame_id = "base_link";
      point_cloud_pub_->publish(out_aw);
    }
    {
      sensor_msgs::msg::PointCloud2 out_ls;
      pcl::toROSMsg(*cloud_liosam, out_ls);
      out_ls.header.stamp    = msg->header.stamp;
      out_ls.header.frame_id = "base_link";
      point_cloud_pub_xyzirt_->publish(out_ls);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_xyzirt_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  geometry_msgs::msg::TransformStamped transform_;
  double min_height_threshold = -1.0;
  double max_height_threshold = 1.0;
  bool use_height_filter = false;
  int ring_offset_  = -1;
  int time_offset_  = -1;
  int time_datatype_ = -1;
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(PointCloudTransformer2)

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudTransformer2>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

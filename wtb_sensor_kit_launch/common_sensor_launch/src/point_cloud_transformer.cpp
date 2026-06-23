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
      rclcpp::QoS(5).best_effort().durability_volatile());

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
    // Resolve ring offset once (Leishen C16 uses "ring", others may use "channel")
    if (ring_offset_ < 0) {
      ring_offset_ = field_offset(*msg, "ring");
      if (ring_offset_ < 0) ring_offset_ = field_offset(*msg, "channel");
    }

    // Step 1: read input as PointXYZI (x, y, z, intensity as float)
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>);
    try {
      pcl::fromROSMsg(*msg, *cloud_in);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to convert input PointCloud2: %s", e.what());
      return;
    }

    // Step 2: create Autoware-compatible output (push_back avoids ghost points from filter)
    auto cloud_out = std::make_shared<pcl::PointCloud<PointXYZIRC>>();
    cloud_out->header = cloud_in->header;
    cloud_out->height = 1;
    cloud_out->is_dense = cloud_in->is_dense;
    cloud_out->points.reserve(cloud_in->points.size());

    // Step 3: transform matrix
    Eigen::Matrix4f T = tf2::transformToEigen(transform_.transform).matrix().cast<float>();

    // Step 4: transform each point
    const auto * raw_data = msg->data.data();
    const int point_step = static_cast<int>(msg->point_step);

    for (size_t i = 0; i < cloud_in->points.size(); ++i) {
      const auto & p_in = cloud_in->points[i];

      Eigen::Vector4f pt(p_in.x, p_in.y, p_in.z, 1.0f);
      Eigen::Vector4f pt_t = T * pt;

      if (use_height_filter) {
        if (pt_t.z() < min_height_threshold || pt_t.z() > max_height_threshold) {
          continue;
        }
      }

      PointXYZIRC p_out;
      p_out.x = pt_t.x();
      p_out.y = pt_t.y();
      p_out.z = pt_t.z();
      p_out.intensity = static_cast<std::uint8_t>(
          std::min(255.0f, std::max(0.0f, p_in.intensity)));
      p_out.return_type = static_cast<std::uint8_t>(
          autoware::point_types::ReturnType::SINGLE_STRONGEST);
      p_out.channel = read_uint16(raw_data + i * point_step, ring_offset_);

      cloud_out->points.push_back(p_out);
    }

    cloud_out->width = cloud_out->points.size();

    // Step 5: publish
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*cloud_out, output_msg);
    output_msg.header.stamp = msg->header.stamp;
    output_msg.header.frame_id = "base_link";

    point_cloud_pub_->publish(output_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  geometry_msgs::msg::TransformStamped transform_;
  double min_height_threshold = -1.0;
  double max_height_threshold = 1.0;
  bool use_height_filter = false;
  int ring_offset_ = -1;
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

#include <chrono>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

int main() {
  constexpr std::size_t points = 9156396;
  pcl::PointCloud<pcl::PointXYZINormal> cloud;
  cloud.resize(points);
  cloud.header.frame_id = "audit_no_ros_node";
  sensor_msgs::msg::PointCloud2 message;
  const auto started = std::chrono::steady_clock::now();
  pcl::toROSMsg(cloud, message);
  const auto converted = std::chrono::steady_clock::now();
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;
  rclcpp::SerializedMessage serialized;
  serializer.serialize_message(&message, &serialized);
  const auto finished = std::chrono::steady_clock::now();
  const double conversion = std::chrono::duration<double>(converted - started).count();
  const double serialization = std::chrono::duration<double>(finished - converted).count();
  std::cout << "{\"points\":" << points
            << ",\"message_bytes\":" << message.data.size()
            << ",\"serialized_bytes\":" << serialized.size()
            << ",\"conversion_sec\":" << conversion
            << ",\"serialization_sec\":" << serialization
            << ",\"total_sec\":" << conversion + serialization
            << ",\"imu_queue_window_sec\":0.05,\"ros_node_created\":false}"
            << std::endl;
}

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <hector_grid_map_compression/msg/compressed_grid_map.hpp>

class ImageToMap : public rclcpp::Node
{
public:
  ImageToMap();
  ~ImageToMap() = default;

private:
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr decompressed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
  rclcpp::Subscription<hector_grid_map_compression::msg::CompressedGridMap>::SharedPtr compressed_sub_;

  grid_map::GridMapRosConverter converter_;
  grid_map::GridMap map_;
  cv_bridge::CvImage image_;

  std::vector<std::string> layers_;
  bool map_initialized_;
  bool subscribed_;

  void connectCb();
  void compressedMapCb(const hector_grid_map_compression::msg::CompressedGridMap& msg);
};

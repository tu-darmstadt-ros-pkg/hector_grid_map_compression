#pragma once

#include <rclcpp/rclcpp.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>

#include "hector_grid_map_compression/msg/compressed_grid_map.hpp"

class MapToImage : public rclcpp::Node
{
public:
  MapToImage();
  ~MapToImage() = default;

private:
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr map_sub_;
  rclcpp::Publisher<hector_grid_map_compression::msg::CompressedGridMap>::SharedPtr compressed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr img_pub_compr_;

  grid_map::GridMapRosConverter converter_;

  //  std::string layer_;
  std::vector<std::string> layers_;
  bool subscribed_;

  void connectCb();
  void mapCb(const grid_map_msgs::msg::GridMap& msg);
};

#pragma once

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <hector_grid_map_compression_msgs/CompressedGridMap.h>
#include <hector_grid_map_compression/compression.h>

namespace hector_grid_map_compression
{

class Client
{
public:
  Client();
  ~Client() = default;

private:
  ros::NodeHandle nh_;
  ros::Publisher decompressed_pub_;
  ros::Publisher img_pub_;
  ros::Subscriber compressed_sub_;

  hector_grid_map_compression::Compression compression_;
  grid_map::GridMapRosConverter converter_;
  grid_map::GridMap map_;
  cv_bridge::CvImage image_;

  std::vector<std::string> layers_;
  bool map_initialized_;
  bool subscribed_;

  void connectCb();
  void compressedMapCb(const hector_grid_map_compression_msgs::CompressedGridMapConstPtr& msg);
};
}  // namespace hector_grid_map_compression
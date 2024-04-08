#pragma once

#include <ros/ros.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <hector_grid_map_compression/compression.h>

namespace hector_grid_map_compression
{

class Server
{
public:
  Server(ros::NodeHandle& nh_,ros::NodeHandle& pnh_);
  ~Server() = default;

private:
  ros::NodeHandle nh_;
  ros::Subscriber map_sub_;
  ros::Publisher compressed_pub_;
  ros::Publisher img_pub_;
  ros::Publisher img_pub_compr_;

  hector_grid_map_compression::Compression compression_;
  grid_map::GridMapRosConverter converter_;

  std::vector<std::string> layers_;
  bool subscribed_;

  void connectCb();
  void mapCb(const grid_map_msgs::GridMapConstPtr& msg);
};
}  // namespace hector_grid_map_compression
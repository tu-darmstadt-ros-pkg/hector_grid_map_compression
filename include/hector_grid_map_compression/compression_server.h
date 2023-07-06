#pragma once

#include <ros/ros.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <image_transport/image_transport.h>

class MapToImage
{
public:
  MapToImage();
  ~MapToImage() = default;

private:
  ros::NodeHandle nh_;
  ros::Subscriber map_sub_;
  image_transport::ImageTransport it_;
  image_transport::Publisher image_pub_;

  grid_map::GridMapRosConverter converter_;

  std::string layer_;
  bool subscribed_;

  void connectCb();
  void mapCb(const grid_map_msgs::GridMapConstPtr& msg);
};

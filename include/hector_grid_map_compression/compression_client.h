#pragma once

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>

class ImageToMap
{
public:
  ImageToMap();
  ~ImageToMap() = default;

private:
  ros::NodeHandle nh_;
  ros::Publisher map_pub_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

  grid_map::GridMapRosConverter converter_;
  grid_map::GridMap map_;
  cv_bridge::CvImage image_;

  std::string layer_;
  bool subscribed_;

  void connectCb();
  void callback(const sensor_msgs::ImageConstPtr& msg);
};

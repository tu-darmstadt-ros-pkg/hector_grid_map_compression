#pragma once

#include <ros/ros.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>

class MapToImage
{
public:
  MapToImage();
  ~MapToImage() = default;

private:
  ros::NodeHandle nh_;
  ros::Subscriber map_sub_;
  //  image_transport::ImageTransport it_;
  //  image_transport::Publisher image_pub_;
  ros::Publisher compressed_pub_;
  //  ros::Publisher img_pub_;
  //  ros::Publisher img_pub_compr_;

  grid_map::GridMapRosConverter converter_;

  //  std::string layer_;
  std::vector<std::string> layers_;
  bool subscribed_;

  void connectCb();
  void mapCb(const grid_map_msgs::GridMapConstPtr& msg);
};

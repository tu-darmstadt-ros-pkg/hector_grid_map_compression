#include <ros/ros.h>
#include <hector_grid_map_compression/client.h>

using namespace hector_grid_map_compression;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hector_grid_map_compression_client");

  ROS_INFO("[compression_client] Starting compression_client");
  Client client;

  ros::spin();

  return 0;
}
#include <ros/ros.h>
#include <hector_grid_map_compression/server.h>

using namespace hector_grid_map_compression;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hector_grid_map_compression_server");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ROS_INFO("[compression_server] Starting compression_server");
  Server server(nh, pnh);

  ros::spin();

  return 0;
}

#include <hector_grid_map_compression/server.h>
#include <hector_grid_map_compression_msgs/CompressedGridLayer.h>
#include <hector_grid_map_compression_msgs/CompressedGridMap.h>

namespace hector_grid_map_compression
{

Server::Server(ros::NodeHandle& nh_,ros::NodeHandle& pnh_) : nh_(nh_)
{

  ros::SubscriberStatusCallback connect_cb = boost::bind(&Server::connectCb, this);
  map_sub_ = nh_.subscribe<grid_map_msgs::GridMap>("input", 1, &Server::mapCb, this);
  compressed_pub_ = nh_.advertise<hector_grid_map_compression_msgs::CompressedGridMap>("output", 1, connect_cb, connect_cb);
  ROS_INFO("[compression_server] Publishing to %s", compressed_pub_.getTopic().c_str());
  ROS_INFO("[compression_server] Subscribing to %s", map_sub_.getTopic().c_str());

  if (!nh_.getParam("layers", layers_))
    ROS_WARN("[compression_server] No layer specified, compressing all available layers");

  subscribed_ = false;
  connectCb();
}

void Server::connectCb()
{
  ROS_INFO("[compression_server] Connected subscribers: %d", compressed_pub_.getNumSubscribers());
  if (compressed_pub_.getNumSubscribers() == 0)
  {
    map_sub_.shutdown();
    subscribed_ = false;
    ROS_INFO("[compression_server] Unsubscribing");
  }
  else
  {
    if (!subscribed_)
    {
      map_sub_ = nh_.subscribe<grid_map_msgs::GridMap>("input", 1, &Server::mapCb, this);    
      subscribed_ = true;
      ROS_INFO("[compression_server] Subscribing");
    }
  }
}

void Server::mapCb(const grid_map_msgs::GridMapConstPtr& in_msg)
{
  hector_grid_map_compression_msgs::CompressedGridMap out_msg;
  compression_.compress(in_msg, out_msg, layers_);

  compressed_pub_.publish(out_msg);
  ROS_INFO_THROTTLE(10, "[compression_server] Publishing compressed map");
}
}  // namespace hector_grid_map_compression
#include <hector_grid_map_compression/client.h>
#include <hector_grid_map_compression_msgs/CompressedGridLayer.h>
#include <opencv2/imgcodecs.hpp>

namespace hector_grid_map_compression
{

Client::Client()
{
  ros::SubscriberStatusCallback connect_cb = boost::bind(&Client::connectCb, this);
  decompressed_pub_ = nh_.advertise<grid_map_msgs::GridMap>("output", 1, connect_cb, connect_cb);
  compressed_sub_ =
      nh_.subscribe<hector_grid_map_compression_msgs::CompressedGridMap>("input", 1, &Client::compressedMapCb, this);
  // ROS_INFO("[compression_client] Publishing to %s", decompressed_pub_.getTopic().c_str());
  // ROS_INFO("[compression_client] Subscribing to %s", compressed_sub_.getTopic().c_str());
  subscribed_ = false;
  map_initialized_ = false;
  connectCb();
}

void Client::connectCb()
{
  // ROS_INFO("[compression_client] Connected subscribers: %d", decompressed_pub_.getNumSubscribers());
  if (decompressed_pub_.getNumSubscribers() == 0)
  {
    compressed_sub_.shutdown();
    subscribed_ = false;
    // ROS_INFO("[compression_client] Unsubscribing");
  }
  else
  {
    if (!subscribed_)
    {
      compressed_sub_ = nh_.subscribe("input", 1, &Client::compressedMapCb, this);
      // ROS_INFO("[compression_client] Subscribing");
      subscribed_ = true;
    }
  }  
}

void Client::compressedMapCb(const hector_grid_map_compression_msgs::CompressedGridMapConstPtr& compressed_map_msg)
{
  
  grid_map_msgs::GridMap out_msg;
  compression_.decompress(compressed_map_msg, out_msg);
  
  decompressed_pub_.publish(out_msg);
  // ROS_INFO_THROTTLE(5, "[compression_client] Publishing decompressed map");
}
}  // namespace hector_grid_map_compression

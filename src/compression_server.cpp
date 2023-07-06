#include <hector_grid_map_compression/compression_server.h>

MapToImage::MapToImage() : nh_("~"), it_(nh_)
{
  image_transport::SubscriberStatusCallback connect_cb = boost::bind(&MapToImage::connectCb, this);
  map_sub_ = nh_.subscribe<grid_map_msgs::GridMap>("input", 1, &MapToImage::mapCb, this);
  image_pub_ = it_.advertise("output", 1, connect_cb, connect_cb);
  if (!nh_.getParam("layer", layer_))
    ROS_ERROR("[ImageToMap] No layer specified");

  subscribed_ = false;
  connectCb();
}

void MapToImage::connectCb()
{
  if (image_pub_.getNumSubscribers() == 0)
  {
    map_sub_.shutdown();
    subscribed_ = false;
    ROS_INFO("Unsubscribing");
  }
  else
  {
    map_sub_ = nh_.subscribe<grid_map_msgs::GridMap>("input", 1, &MapToImage::mapCb, this);
    if (!subscribed_)
      ROS_INFO("Subscribing");
    subscribed_ = true;
  }
}

void MapToImage::mapCb(const grid_map_msgs::GridMapConstPtr& msg)
{
  grid_map::GridMap map;
  cv_bridge::CvImage image;

  // Convert the map msg to grid map
  grid_map::GridMapRosConverter::fromMessage(*msg, map, { layer_ });

  // The GridMapRosConverter::toCvImage method below assings a value
  // To distinguish between invalid and ground pixels on the client side, we set all invalid points to max+1m
  // The client node receives the max valid height and sets all values above that to invalid
  auto high = map.get(layer_).maxCoeffOfFinites();
  auto low = map.get(layer_).minCoeffOfFinites();
  double invalid_val = high + (high - low) / 255;  // max 8bit value

  // Replace nans with invalid_val
  grid_map::Matrix& data = map[layer_];
  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator)
  {
    const int i = iterator.getLinearIndex();
    if (std::isnan(data(i)))
      data(i) = invalid_val;
  }

  // Pack all metadata in the frame_id string that are required for uncompressing
  std::stringstream ss;
  ss << msg->info.header.frame_id << " ";
  ss << msg->info.length_x << " ";
  ss << msg->info.length_y << " ";
  ss << msg->info.pose.position.x << " ";
  ss << msg->info.pose.position.y << " ";
  ss << msg->info.pose.position.z << " ";
  ss << msg->info.pose.orientation.x << " ";
  ss << msg->info.pose.orientation.y << " ";
  ss << msg->info.pose.orientation.z << " ";
  ss << msg->info.pose.orientation.w << " ";
  ss << msg->info.resolution << " ";
  ss << high << " ";
  ss << low;

  grid_map::GridMapRosConverter::toCvImage(map, layer_, sensor_msgs::image_encodings::MONO8, low, invalid_val, image);

  sensor_msgs::Image img_msg = *image.toImageMsg();
  img_msg.header.frame_id = ss.str();
  image_pub_.publish(img_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "elevation_map_compression_server");

  ROS_INFO("Starting compression_server");
  MapToImage map_to_image;

  ros::spin();

  return 0;
}

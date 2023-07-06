#include <hector_grid_map_compression/compression_client.h>

ImageToMap::ImageToMap() : nh_("~"), it_(nh_)
{
  ros::SubscriberStatusCallback connect_cb = boost::bind(&ImageToMap::connectCb, this);
  map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("output", 1, connect_cb, connect_cb);
  image_sub_ = it_.subscribe("input", 1, &ImageToMap::callback, this);
  if (!nh_.getParam("layer", layer_))
    ROS_ERROR("[ImageToMap] No layer specified");

  subscribed_ = false;
  connectCb();

  map_.setBasicLayers({ layer_ });
}

void ImageToMap::connectCb()
{
  if (map_pub_.getNumSubscribers() == 0)
  {
    image_sub_.shutdown();
    subscribed_ = false;
    ROS_INFO("Unsubscribing");
  }
  else
  {
    image_sub_ = it_.subscribe("input", 1, &ImageToMap::callback, this);
    if (!subscribed_)
      ROS_INFO("Subscribing");
    subscribed_ = true;
  }
}

void ImageToMap::callback(const sensor_msgs::ImageConstPtr& msg)
{
  // get metadata from frame_id
  // frame_id length_x length_y x y z rx ry rz rw resolution high low
  std::string metadata = msg->header.frame_id;
  std::stringstream ss(msg->header.frame_id);
  std::vector<std::string> split;
  std::string item;
  while (std::getline(ss, item, ' '))
  {
    split.push_back(item);
  }
  if (split.size() != 13)
  {
    ROS_ERROR("Cannot extract meta data from frame_id! Expected %d items, got %zu", 14, split.size());
    return;
  }

  std::string frame_id;
  double length_x, length_y, p_x, p_y, p_z, o_x, o_y, o_z, o_w, resolution, high, low;
  try
  {
    frame_id = split[0];
    length_x = std::stod(split[1]);
    length_y = std::stod(split[2]);
    p_x = std::stod(split[3]);
    p_y = std::stod(split[4]);
    p_z = std::stod(split[5]);
    o_x = std::stod(split[7]);
    o_y = std::stod(split[7]);
    o_z = std::stod(split[8]);
    o_w = std::stod(split[9]);
    resolution = std::stod(split[10]);
    high = std::stod(split[11]);
    low = std::stod(split[12]);
  }
  catch (const std::invalid_argument&)
  {
    std::cerr << "Error while parsing meta data from frame_id string\n";
    throw;
  }

  double invalid_val = high + (high - low) / 255;
  grid_map::GridMapRosConverter::initializeFromImage(*msg, resolution, map_);
  grid_map::GridMapRosConverter::addLayerFromImage(*msg, layer_, map_, low, invalid_val);
  grid_map::Matrix& data = map_[layer_];

  // All grid cells with value > info->high are invalid
  for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator)
  {
    const int i = iterator.getLinearIndex();
    if (data(i) > high)
      data(i) = std::numeric_limits<double>::quiet_NaN();
  }

  // Publish as grid map
  grid_map_msgs::GridMap map_msg;
  grid_map::GridMapRosConverter::toMessage(map_, map_msg);
  map_msg.info.header.frame_id = frame_id;
  map_msg.info.length_x = length_x;
  map_msg.info.length_y = length_y;
  map_msg.info.pose.position.x = p_x;
  map_msg.info.pose.position.y = p_y;
  map_msg.info.pose.position.z = p_z;
  map_msg.info.pose.orientation.x = o_x;
  map_msg.info.pose.orientation.y = o_y;
  map_msg.info.pose.orientation.z = o_z;
  map_msg.info.pose.orientation.w = o_w;
  map_msg.info.resolution = resolution;
  map_pub_.publish(map_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hector_grid_map_compression_client");

  ROS_INFO("Starting compression_client");
  ImageToMap image_to_map;

  ros::spin();

  return 0;
}

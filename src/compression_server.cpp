#include <hector_grid_map_compression/compression_server.h>
//#include <hector_grid_map_compression/CompressedGridLayer.h>
//#include <hector_grid_map_compression/CompressedGridMap.h>
#include <hector_grid_map_compression/GridLayer.h>
#include <hector_grid_map_compression/GridMap.h>

MapToImage::MapToImage() : nh_("~")
{
  map_sub_ = nh_.subscribe<grid_map_msgs::GridMap>("input", 1, &MapToImage::mapCb, this);
  compressed_pub_ = nh_.advertise<hector_grid_map_compression::GridMap>("output", 1);
  //  img_pub_ = nh_.advertise<sensor_msgs::Image>("server_img", 1);
  //  img_pub_compr_ = nh_.advertise<sensor_msgs::CompressedImage>("server_img/compressed", 1);
  if (!nh_.getParam("layers", layers_))
    ROS_WARN("[ImageToMap] No layer specified, compressing all available layers");

  subscribed_ = false;
  connectCb();
}

void MapToImage::connectCb()
{
  if (false && compressed_pub_.getNumSubscribers() == 0)
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

void MapToImage::mapCb(const grid_map_msgs::GridMapConstPtr& in_msg)
{
  grid_map::GridMap map;
  cv_bridge::CvImage image;
  hector_grid_map_compression::GridMap map_msg;

  // Convert the map msg to grid map
  std::vector<std::string> available_layers = in_msg->layers;
  grid_map::GridMapRosConverter::fromMessage(*in_msg, map, available_layers);

  // Fill msg header and info
  map_msg.header = in_msg->info.header;
  map_msg.info = in_msg->info;

  //  for (const auto& layer : layers_)
  std::cout << "\n";
  for (const auto& layer : available_layers)
  {
    std::cout << "layer: " << layer << "\n";
    if (std::find(layers_.begin(), layers_.end(), layer) != layers_.end() & !layers_.empty())
    {
      std::cout << "skipped\n";
      continue;
    }

    // The GridMapRosConverter::toCvImage method below assings a value
    // To distinguish between invalid and ground pixels on the client side, we set all invalid points to max+1m
    // The client node receives the max valid height and sets all values above that to invalid
    double high = map.get(layer).maxCoeffOfFinites();
    double low = map.get(layer).minCoeffOfFinites();
    double invalid_val = high + (high - low) / 255;  // max 8bit value

    // Replace nans with invalid_val
    grid_map::Matrix& data = map[layer];
    for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator)
    {
      const int i = iterator.getLinearIndex();
      if (std::isnan(data(i)))
        data(i) = invalid_val;
    }

    // Create msg for current layer
    hector_grid_map_compression::GridLayer layer_msg;
    layer_msg.layer.header = in_msg->info.header;
    layer_msg.min_val = low;
    layer_msg.max_val = high;
    layer_msg.name = layer;

    // Convert map to img
    grid_map::GridMapRosConverter::toCvImage(map, layer, sensor_msgs::image_encodings::MONO8, low, invalid_val, image);
    layer_msg.layer = *image.toImageMsg();

    map_msg.layers.push_back(layer_msg);
  }

  std::cout << "publishing " << map_msg.layers.size() << " layers, frame_id " << map_msg.header.frame_id << "\n";
  compressed_pub_.publish(map_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hector_grid_map_compression_server");

  ROS_INFO("Starting compression_server");
  MapToImage map_to_image;

  ros::spin();

  return 0;
}

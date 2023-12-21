#include <hector_grid_map_compression/compression_server.h>
#include <hector_grid_map_compression/CompressedGridLayer.h>
#include <hector_grid_map_compression/CompressedGridMap.h>

MapToImage::MapToImage() : nh_("~")
{
  //  image_transport::SubscriberStatusCallback connect_cb = boost::bind(&MapToImage::connectCb, this);
  map_sub_ = nh_.subscribe<grid_map_msgs::GridMap>("input", 1, &MapToImage::mapCb, this);
  compressed_pub_ = nh_.advertise<hector_grid_map_compression::CompressedGridMap>("output", 1);
  img_pub_ = nh_.advertise<sensor_msgs::Image>("server_img", 1);
  img_pub_compr_ = nh_.advertise<sensor_msgs::CompressedImage>("server_img/compressed", 1);
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
  hector_grid_map_compression::CompressedGridMap compressed_map_msg;

  // Convert the map msg to grid map
  std::vector<std::string> available_layers = in_msg->layers;
  grid_map::GridMapRosConverter::fromMessage(*in_msg, map, available_layers);

  // Fill msg header and info
  compressed_map_msg.header = in_msg->info.header;
  compressed_map_msg.info = in_msg->info;

  for (const auto& layer : available_layers)
  {
    if (std::find(layers_.begin(), layers_.end(), layer) != layers_.end() & !layers_.empty())
      continue;

    // The GridMapRosConverter::toCvImage method below assings a value
    // To distinguish between invalid and ground pixels on the client side, we set all invalid points to max+1m
    // The client node receives the max valid height and sets all values above that to invalid.
    //
    // Because of lossy compression (jpg) there will be artifacts around invalid points.
    // If max+1m is too close to max the client node mistakes some invalid point artifacts for max
    // => use max+(1m x 10) for invalid points for a bigger difference to max.
    double high = map.get(layer).maxCoeffOfFinites();
    double low = map.get(layer).minCoeffOfFinites();
    double invalid_val = high + ((high - low) / 255) * 10;  // max 8bit value

    // Replace nans with invalid_val
    grid_map::Matrix& data = map[layer];
    for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator)
    {
      const int i = iterator.getLinearIndex();
      if (std::isnan(data(i)))
        data(i) = invalid_val;
    }

    // Create msg for current layer
    hector_grid_map_compression::CompressedGridLayer layer_msg;
    layer_msg.layer.header = in_msg->info.header;
    layer_msg.min_val = low;
    layer_msg.max_val = high;
    layer_msg.name = layer;

    // Convert map to img
    grid_map::GridMapRosConverter::toCvImage(map, layer, sensor_msgs::image_encodings::MONO8, low, invalid_val, image);
    img_pub_.publish(image.toImageMsg());

    // Compression settings
    std::vector<int> params;
    params.reserve(2);
    params.push_back(cv::IMWRITE_JPEG_QUALITY);
    params.push_back(100);  // jpeg quality from 0 to 100

    // Compress img
    layer_msg.layer.header = in_msg->info.header;
    layer_msg.layer.format = "jpeg";
    try
    {
      cv::imencode(".jpg", image.image, layer_msg.layer.data, params);
      //      float cRatio =
      //          (float)compressed.data.size() / (float)(image.image.rows * image.image.cols * image.image.elemSize());
      //    std::cout << " compression ratio: " << cRatio << "\n";
      img_pub_compr_.publish(layer_msg.layer);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_WARN("Failed to compress image! %s", e.what());
    }
    catch (cv::Exception& e)
    {
      ROS_WARN("Failed to compress image! %s", e.what());
    }
    compressed_map_msg.layers.push_back(layer_msg);
  }

  compressed_pub_.publish(compressed_map_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hector_grid_map_compression_server");

  ROS_INFO("Starting compression_server");
  MapToImage map_to_image;

  ros::spin();

  return 0;
}

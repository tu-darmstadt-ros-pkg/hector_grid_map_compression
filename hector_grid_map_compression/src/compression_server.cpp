#include <hector_grid_map_compression/compression_server.h>
#include <hector_grid_map_compression_msgs/CompressedGridLayer.h>
#include <hector_grid_map_compression_msgs/CompressedGridMap.h>

Compression::Compression() : nh_("~")
{
  ros::SubscriberStatusCallback connect_cb = boost::bind(&Compression::connectCb, this);
  map_sub_ = nh_.subscribe<grid_map_msgs::GridMap>("input", 1, &Compression::mapCb, this);
  compressed_pub_ = nh_.advertise<hector_grid_map_compression_msgs::CompressedGridMap>("output", 1, connect_cb, connect_cb);
  ROS_INFO("[compression_server] Publishing to %s", compressed_pub_.getTopic().c_str());
  ROS_INFO("[compression_server] Subscribing to %s", map_sub_.getTopic().c_str());
  // img_pub_ = nh_.advertise<sensor_msgs::Image>("server_img", 1);
  // img_pub_compr_ = nh_.advertise<sensor_msgs::CompressedImage>("server_img/compressed", 1);
  if (!nh_.getParam("layers", layers_))
    ROS_WARN("[compression_server]  No layer specified, compressing all available layers");

  subscribed_ = false;
  connectCb();
}

void Compression::connectCb()
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
      map_sub_ = nh_.subscribe<grid_map_msgs::GridMap>("input", 1, &Compression::mapCb, this);    
      subscribed_ = true;
      ROS_INFO("[compression_server] Subscribing");
    }
  }
}

void Compression::mapCb(const grid_map_msgs::GridMapConstPtr& in_msg)
{
  grid_map::GridMap map;
  cv_bridge::CvImage image;
  hector_grid_map_compression_msgs::CompressedGridMap compressed_map_msg;

  // Convert the map msg to grid map
  std::vector<std::string> available_layers = in_msg->layers;
  grid_map::GridMapRosConverter::fromMessage(*in_msg, map, available_layers);

  // Fill msg header and info
  compressed_map_msg.header = in_msg->info.header;
  compressed_map_msg.info = in_msg->info;

  for (const auto& layer : available_layers)
  {
    if (std::find(layers_.begin(), layers_.end(), layer) == layers_.end() & !layers_.empty())
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
    hector_grid_map_compression_msgs::CompressedGridLayer layer_msg;
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
      ROS_WARN("[compression_server] Failed to compress image! %s", e.what());
    }
    catch (cv::Exception& e)
    {
      ROS_WARN("[compression_server] Failed to compress image! %s", e.what());
    }
    compressed_map_msg.layers.push_back(layer_msg);
  }

  compressed_pub_.publish(compressed_map_msg);
  std::stringstream ss;
  for (const auto& layer : compressed_map_msg.layers)
    ss << "<" << layer.name << "> " << layer.layer.data.size() << " bytes ";
  ROS_INFO_THROTTLE(5, "[compression_server] Publishing compressed map, sizes: %s", ss.str().c_str());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hector_grid_map_compression_server");

  ROS_INFO("[compression_server] Starting compression_server");
  Compression compression;

  ros::spin();

  return 0;
}

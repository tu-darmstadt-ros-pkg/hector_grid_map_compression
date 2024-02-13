#include <hector_grid_map_compression/compression_server.h>
#include <hector_grid_map_compression/msg/compressed_grid_layer.hpp>

MapToImage::MapToImage() : Node("hector_grid_map_compression_server")
{
  auto map_cb = std::bind(&MapToImage::mapCb, this, std::placeholders::_1);
  map_sub_ = this->create_subscription<grid_map_msgs::msg::GridMap>("input", 1, map_cb);
  compressed_pub_ = this->create_publisher<hector_grid_map_compression::msg::CompressedGridMap>("output", 1);
  img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("server_img", 1);
  img_pub_compr_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("server_img/compressed", 1);
  this->declare_parameter("layers", rclcpp::PARAMETER_STRING_ARRAY);
  try
  {
    layers_ = this->get_parameter("layers").as_string_array();
  }
  catch (const rclcpp::exceptions::ParameterUninitializedException& e)
  {
    e.what();
    RCLCPP_WARN(this->get_logger(), "[ImageToMap] No layer specified, compressing all available layers");
  }
}

void MapToImage::mapCb(const grid_map_msgs::msg::GridMap& in_msg)
{
  // In ROS1 we can use the SubscriberStatusCallback to lazy connect to this callback,
  // but in ROS2 there is no SubscriberStatusCallback yet.
  // Avoid unnecessary computation and bandwith consumption by returning.
  if (compressed_pub_->get_subscription_count() == 0)
  {
    RCLCPP_INFO(this->get_logger(), "No subscriptions. Don't compress.");
    return;
  }

  grid_map::GridMap map;
  cv_bridge::CvImage image;
  hector_grid_map_compression::msg::CompressedGridMap compressed_map_msg;

  // Convert the map msg to grid map
  std::vector<std::string> available_layers = in_msg.layers;
  grid_map::GridMapRosConverter::fromMessage(in_msg, map, available_layers);

  // Fill msg header and info
  compressed_map_msg.header = in_msg.header;
  compressed_map_msg.info = in_msg.info;

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
    hector_grid_map_compression::msg::CompressedGridLayer layer_msg;
    layer_msg.layer.header = in_msg.header;
    layer_msg.min_val = low;
    layer_msg.max_val = high;
    layer_msg.name = layer;

    // Convert map to img
    grid_map::GridMapRosConverter::toCvImage(map, layer, sensor_msgs::image_encodings::MONO8, low, invalid_val, image);
    img_pub_->publish(*image.toImageMsg());

    // Compression settings
    std::vector<int> params;
    params.reserve(2);
    params.push_back(cv::IMWRITE_JPEG_QUALITY);
    params.push_back(100);  // jpeg quality from 0 to 100

    // Compress img
    layer_msg.layer.header = in_msg.header;
    layer_msg.layer.format = "jpeg";
    try
    {
      cv::imencode(".jpg", image.image, layer_msg.layer.data, params);
      //      float cRatio =
      //          (float)compressed.data.size() / (float)(image.image.rows * image.image.cols * image.image.elemSize());
      //    std::cout << " compression ratio: " << cRatio << "\n";
      img_pub_compr_->publish(layer_msg.layer);
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_WARN(this->get_logger(), "Failed to compress image! %s", e.what());
    }
    catch (cv::Exception& e)
    {
      RCLCPP_WARN(this->get_logger(), "Failed to compress image! %s", e.what());
    }
    compressed_map_msg.layers.push_back(layer_msg);
  }

  compressed_pub_->publish(compressed_map_msg);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting compression_server");

  rclcpp::spin(std::make_shared<MapToImage>());
  rclcpp::shutdown();

  return 0;
}

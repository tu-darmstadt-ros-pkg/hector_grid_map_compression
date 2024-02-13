#include <hector_grid_map_compression/compression_client.h>
#include <hector_grid_map_compression/msg/compressed_grid_layer.hpp>
//#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

using CompressedGridMapMsg = hector_grid_map_compression::msg::CompressedGridMap;

ImageToMap::ImageToMap() : Node("hector_grid_map_compression_client")
{
  decompressed_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("output", 1);
  img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("debug_img", 1);
  auto compressed_cb = std::bind(&ImageToMap::compressedMapCb, this, std::placeholders::_1);
  compressed_sub_ = this->create_subscription<CompressedGridMapMsg>("input", 1, compressed_cb);

  // TODO there is no SubscriberStatusCallback in ROS2 yet
  // subscribed_ = false;
  map_initialized_ = false;
  // connectCb();

  //  map_.setBasicLayers({ layer_ });
}

void ImageToMap::connectCb()
{
  if (decompressed_pub_->get_subscription_count() == 0)
  {
    compressed_sub_.reset();
    subscribed_ = false;
    RCLCPP_INFO(this->get_logger(), "Unsubscribing");
  }
  else
  {
    auto compressed_cb = std::bind(&ImageToMap::compressedMapCb, this, std::placeholders::_1);
    compressed_sub_ = this->create_subscription<CompressedGridMapMsg>("input", 1, compressed_cb);
    if (!subscribed_)
      RCLCPP_INFO(this->get_logger(), "Subscribing");
    subscribed_ = true;
  }
}

void ImageToMap::compressedMapCb(const CompressedGridMapMsg& compressed_map_msg)
{
  if (!map_initialized_)
  {
    // Store all available layers
    for (const auto& layer : compressed_map_msg.layers)
      layers_.push_back(layer.name);
    if (layers_.empty())
    {
      RCLCPP_WARN(this->get_logger(), "Received message contains no layers");
      return;
    }
  }

  for (const auto& layer_msg : compressed_map_msg.layers)
  {
    // Uncompress image
    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
    try
    {
      cv_ptr->image = cv::imdecode(cv::Mat(layer_msg.layer.data), cv::IMREAD_GRAYSCALE);
    }
    catch (cv::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to decompress image msg! %s", e.what());
      return;
    }
    std::shared_ptr<sensor_msgs::msg::Image> img_decompressed = cv_ptr->toImageMsg();
    img_decompressed->encoding = "mono8";

    if (layer_msg.name == "elevation")
      img_pub_->publish(*img_decompressed);

    // Initialize
    if (!map_initialized_)
    {
      grid_map::GridMapRosConverter::initializeFromImage(*img_decompressed, compressed_map_msg.info.resolution, map_);
      map_initialized_ = true;
    }

    double invalid_val = layer_msg.max_val + ((layer_msg.max_val - layer_msg.min_val) / 255) * 10;
    grid_map::GridMapRosConverter::addLayerFromImage(*img_decompressed, layer_msg.name, map_, layer_msg.min_val,
                                                     invalid_val);
    grid_map::Matrix& data = map_[layer_msg.name];

    // All grid cells with value > max_val are invalid
    for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator)
    {
      const int i = iterator.getLinearIndex();
      if (data(i) > layer_msg.max_val)
      {
        data(i) = std::numeric_limits<double>::quiet_NaN();
      }
    }
  }

  // Publish as grid map
  std::unique_ptr<grid_map_msgs::msg::GridMap> map_msg = grid_map::GridMapRosConverter::toMessage(map_);
  map_msg->basic_layers.push_back("elevation");
  map_msg->info = compressed_map_msg.info;
  map_msg->header = compressed_map_msg.header;
  decompressed_pub_->publish(*map_msg);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting compression_client");

  rclcpp::spin(std::make_shared<ImageToMap>());
  rclcpp::shutdown();

  return 0;
}

#include <hector_grid_map_compression/client.h>
#include <hector_grid_map_compression_msgs/CompressedGridLayer.h>
//#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

namespace hector_grid_map_compression
{

Decompression::Decompression() : nh_("~")
{
  ros::SubscriberStatusCallback connect_cb = boost::bind(&Decompression::connectCb, this);
  decompressed_pub_ = nh_.advertise<grid_map_msgs::GridMap>("output", 1, connect_cb, connect_cb);
  // img_pub_ = nh_.advertise<sensor_msgs::Image>("debug_img", 1);  
  compressed_sub_ =
      nh_.subscribe<hector_grid_map_compression_msgs::CompressedGridMap>("input", 1, &Decompression::compressedMapCb, this);
  ROS_INFO("[compression_client] Publishing to %s", decompressed_pub_.getTopic().c_str());
  ROS_INFO("[compression_client] Subscribing to %s", compressed_sub_.getTopic().c_str());
  subscribed_ = false;
  map_initialized_ = false;
  connectCb();

  //  map_.setBasicLayers({ layer_ });
}

void Decompression::connectCb()
{
  ROS_INFO("[compression_client] Connected subscribers: %d", decompressed_pub_.getNumSubscribers());
  if (decompressed_pub_.getNumSubscribers() == 0)
  {
    compressed_sub_.shutdown();
    subscribed_ = false;
    ROS_INFO("[compression_client] Unsubscribing");
  }
  else
  {
    if (!subscribed_)
    {
      compressed_sub_ = nh_.subscribe("input", 1, &Decompression::compressedMapCb, this);
      ROS_INFO("[compression_client] Subscribing");
      subscribed_ = true;
    }
  }  
}

void Decompression::compressedMapCb(const hector_grid_map_compression_msgs::CompressedGridMapConstPtr& compressed_map_msg)
{
  if (!map_initialized_)
  {
    // Store all available layers
    for (const auto& layer : compressed_map_msg->layers)
      layers_.push_back(layer.name);
    if (layers_.empty())
    {
      ROS_WARN("[compression_client] Received message contains no layers");
      return;
    }
  }

  for (const auto& layer_msg : compressed_map_msg->layers)
  {
    // Uncompress image
    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
    try
    {
      cv_ptr->image = cv::imdecode(cv::Mat(layer_msg.layer.data), cv::IMREAD_GRAYSCALE);
    }
    catch (cv::Exception& e)
    {
      ROS_ERROR("[compression_client] Failed to decompress image msg! %s", e.what());
      return;
    }
    const sensor_msgs::ImagePtr img_decompressed = cv_ptr->toImageMsg();
    img_decompressed->encoding = "mono8";

    // if (layer_msg.name == "elevation")
    //   img_pub_.publish(*img_decompressed);

    // Initialize
    if (!map_initialized_)
    {
      grid_map::GridMapRosConverter::initializeFromImage(*img_decompressed, compressed_map_msg->info.resolution, map_);
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
  grid_map_msgs::GridMap map_msg;
  grid_map::GridMapRosConverter::toMessage(map_, map_msg);
  map_msg.basic_layers.push_back("elevation");
  map_msg.info = compressed_map_msg->info;
  decompressed_pub_.publish(map_msg);

  ROS_INFO_THROTTLE(5, "[compression_client] Publishing decompressed map");
}
}  // namespace hector_grid_map_compression

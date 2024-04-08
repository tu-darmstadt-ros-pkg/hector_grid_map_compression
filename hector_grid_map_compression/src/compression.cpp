#include <hector_grid_map_compression/compression.h>
#include <hector_grid_map_compression_msgs/CompressedGridLayer.h>
#include <hector_grid_map_compression_msgs/CompressedGridMap.h>

namespace hector_grid_map_compression
{
  Compression::Compression() : decompression_initialized_(false)
  {
  }

  void Compression::compress(const grid_map_msgs::GridMapConstPtr& orig_msg, hector_grid_map_compression_msgs::CompressedGridMap& compressed_msg, const std::vector<std::string>& layers)
  {
    grid_map::GridMap map;
    cv_bridge::CvImage image;

    // Convert the map msg to grid map
    std::vector<std::string> available_layers = orig_msg->layers;
    grid_map::GridMapRosConverter::fromMessage(*orig_msg, map, available_layers);

    // Fill msg header and info
    compressed_msg.header = orig_msg->info.header;
    compressed_msg.info = orig_msg->info;

    for (const auto& layer : available_layers)
    {
      if (std::find(layers.begin(), layers.end(), layer) == layers.end() & !layers.empty())
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
      layer_msg.layer.header = orig_msg->info.header;
      layer_msg.min_val = low;
      layer_msg.max_val = high;
      layer_msg.name = layer;

      // Convert map to img
      grid_map::GridMapRosConverter::toCvImage(map, layer, sensor_msgs::image_encodings::MONO8, low, invalid_val, image);

      // Server settings
      std::vector<int> params;
      params.reserve(2);
      params.push_back(cv::IMWRITE_JPEG_QUALITY);
      params.push_back(100);  // jpeg quality from 0 to 100

      // Compress img
      layer_msg.layer.header = orig_msg->info.header;
      layer_msg.layer.format = "jpeg";
      try
      {
        cv::imencode(".jpg", image.image, layer_msg.layer.data, params);
        //      float cRatio =
        //          (float)compressed.data.size() / (float)(image.image.rows * image.image.cols * image.image.elemSize());
        //    std::cout << " compression ratio: " << cRatio << "\n";
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_WARN("[compression_server] Failed to compress image! %s", e.what());
      }
      catch (cv::Exception& e)
      {
        ROS_WARN("[compression_server] Failed to compress image! %s", e.what());
      }
      compressed_msg.layers.push_back(layer_msg);
    }
  }

  void Compression::decompress(const hector_grid_map_compression_msgs::CompressedGridMapConstPtr& compressed_msg, grid_map_msgs::GridMap& decompressed_msg)
  {
    for (const auto& layer_msg : compressed_msg->layers)
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

      // Initialize
      if (!decompression_initialized_)
      {
        grid_map::GridMapRosConverter::initializeFromImage(*img_decompressed, compressed_msg->info.resolution, decompressed_map_);
        decompression_initialized_ = true;
      }

      double invalid_val = layer_msg.max_val + ((layer_msg.max_val - layer_msg.min_val) / 255) * 10;
      grid_map::GridMapRosConverter::addLayerFromImage(*img_decompressed, layer_msg.name, decompressed_map_, layer_msg.min_val,
                                                      invalid_val);
      grid_map::Matrix& data = decompressed_map_[layer_msg.name];

      // All grid cells with value > max_val are invalid
      for (grid_map::GridMapIterator iterator(decompressed_map_); !iterator.isPastEnd(); ++iterator)
      {
        const int i = iterator.getLinearIndex();
        if (data(i) > layer_msg.max_val)
        {
          data(i) = std::numeric_limits<double>::quiet_NaN();
        }
      }
    }

    grid_map::GridMapRosConverter::toMessage(decompressed_map_, decompressed_msg);
    decompressed_msg.basic_layers.push_back("elevation");  // TODO
    decompressed_msg.info = compressed_msg->info;

  }
}  // namespace hector_grid_map_compression
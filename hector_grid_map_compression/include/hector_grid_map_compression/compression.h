#pragma once

#include <ros/ros.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <hector_grid_map_compression_msgs/CompressedGridMap.h>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>

namespace hector_grid_map_compression
{

class Compression
{
public:
  Compression();
	~Compression() = default;
	void compress(const grid_map_msgs::GridMapConstPtr& orig_msg, hector_grid_map_compression_msgs::CompressedGridMap& compressed_msg, const std::vector<std::string>& layers);
	void decompress(const hector_grid_map_compression_msgs::CompressedGridMapConstPtr& compressed_msg, grid_map_msgs::GridMap& decompressed_msg);

private:
	grid_map::GridMapRosConverter converter_;
	grid_map::GridMap decompressed_map_;
	bool decompression_initialized_;
};

}  // namespace hector_grid_map_compression


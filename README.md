# hector_grid_map_compression
hector_grid_map_compression provides a server and client to transmit grid maps with high compression as images.

The raw grid_map_msgs/GridMap is very inefficient in terms of storage consumption.
This package provides an implementation to compress grid maps by converting them into compressed JPG images. The compression is lossy, but fairly sufficient for mere visualization of elevation maps.
The compression rate is ~30.

# Usage
On the server side (robot) run
```
roslaunch hector_grid_map_compression server_node.launch
```
Available parameters are `input_topic` (the topic of your grid map), `output_topic` and `layer` (list of all layers you want to include).

On the client side (operator PC) run
```
roslaunch hector_grid_map_compression client_node.launch
```
Make sure that the `input_topic` matches the `output_topic` on the server side.

If you only want to view the map in rviz, you don't need to run the client node. Simply use the CompressedGridMap Display.

# Acknowledgments
This rviz_plugin is adapted from the original [GridMap Rviz Plugin](https://github.com/ANYbotics/grid_map/tree/master/grid_map_rviz_plugin).

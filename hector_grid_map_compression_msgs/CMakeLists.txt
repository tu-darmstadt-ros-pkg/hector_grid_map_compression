cmake_minimum_required(VERSION 3.0.2)
project(hector_grid_map_compression_msgs)

find_package(catkin REQUIRED COMPONENTS
        message_generation
        std_msgs
        sensor_msgs
        grid_map_msgs
        )

add_message_files(
        DIRECTORY msg
        FILES
        CompressedGridLayer.msg
        CompressedGridMap.msg
)


generate_messages(
        DEPENDENCIES std_msgs sensor_msgs grid_map_msgs
)

catkin_package(
        # INCLUDE_DIRS
        CATKIN_DEPENDS message_runtime std_msgs sensor_msgs grid_map_msgs
)

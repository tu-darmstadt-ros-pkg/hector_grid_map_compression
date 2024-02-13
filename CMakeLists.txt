cmake_minimum_required(VERSION 3.8)
project(hector_grid_map_compression)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_cv REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

find_package(OpenCV REQUIRED COMPONENTS
  opencv_highgui
  CONFIG
)

include_directories(include ${OpenCV_INCLUDE_DIRS})

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CompressedGridLayer.msg"
  "msg/CompressedGridMap.msg"
  DEPENDENCIES std_msgs sensor_msgs grid_map_msgs
)
ament_export_dependencies(rosidl_default_runtime)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(hector_grid_map_compression_server src/compression_server.cpp)
ament_target_dependencies(hector_grid_map_compression_server rclcpp image_transport grid_map_msgs grid_map_ros cv_bridge OpenCV)
target_link_libraries(hector_grid_map_compression_server "${cpp_typesupport_target}")

add_executable(hector_grid_map_compression_client src/compression_client.cpp)
ament_target_dependencies(hector_grid_map_compression_client rclcpp image_transport grid_map_ros grid_map_msgs OpenCV)
target_link_libraries(hector_grid_map_compression_client "${cpp_typesupport_target}")

install(TARGETS hector_grid_map_compression_server hector_grid_map_compression_client
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
cmake_minimum_required(VERSION 3.0.2)
project(hector_grid_map_compression)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  grid_map_msgs
  grid_map_core
  grid_map_cv
  grid_map_ros
  eigen_conversions
  cv_bridge
  image_transport  
  message_generation
)

find_package(OpenCV REQUIRED COMPONENTS
  opencv_highgui
  CONFIG
)

include_directories(include ${OpenCV_INCLUDE_DIRS})

add_message_files(
  DIRECTORY msg
  FILES
  CompressedGridLayer.msg
  CompressedGridMap.msg
  GridLayer.msg
  GridMap.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  grid_map_msgs
)

catkin_package(
  INCLUDE_DIRS 
  include
#  LIBRARIES
   CATKIN_DEPENDS roscpp message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(hector_grid_map_compression_server src/compression_server.cpp)
add_dependencies(hector_grid_map_compression_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(hector_grid_map_compression_server ${catkin_LIBRARIES} )

add_executable(hector_grid_map_compression_client src/compression_client.cpp)
add_dependencies(hector_grid_map_compression_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(hector_grid_map_compression_client ${catkin_LIBRARIES} )

install(TARGETS hector_grid_map_compression_server hector_grid_map_compression_client
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY
  launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

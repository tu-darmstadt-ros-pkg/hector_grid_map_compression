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
  hector_grid_map_compression_msgs
  eigen_conversions
  cv_bridge
  image_transport
  nodelet
)

find_package(OpenCV REQUIRED COMPONENTS
  opencv_highgui
  CONFIG
)

include_directories(include ${OpenCV_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hector_grid_map_compression
  CATKIN_DEPENDS 
   roscpp 
   grid_map_ros
   grid_map_msgs
   hector_grid_map_compression_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME} src/client.cpp src/server.cpp src/compression.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_library(${PROJECT_NAME}_nodelet src/server_nodelet.cpp)
target_link_libraries(${PROJECT_NAME}_nodelet ${PROJECT_NAME})

add_executable(hector_grid_map_compression_server src/server_node.cpp)
add_dependencies(hector_grid_map_compression_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(hector_grid_map_compression_server ${PROJECT_NAME} )

add_executable(hector_grid_map_compression_client src/client_node.cpp)
add_dependencies(hector_grid_map_compression_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(hector_grid_map_compression_client ${PROJECT_NAME} )

install(TARGETS hector_grid_map_compression hector_grid_map_compression_server hector_grid_map_compression_client
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ) 

install(DIRECTORY
  launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

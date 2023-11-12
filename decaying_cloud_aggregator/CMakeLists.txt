cmake_minimum_required(VERSION 2.8.3)
project(decaying_cloud_aggregator)

find_package(catkin REQUIRED COMPONENTS
  laser_geometry
  roscpp
  sensor_msgs
  tf
  pcl_ros
)

find_package(PCL REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vigir_cloud_to_planar_scan
#  CATKIN_DEPENDS laser_geometry roscpp sensor_msgs tf
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(decaying_cloud_aggregator_node src/decaying_cloud_aggregator_node.cpp)

target_link_libraries(decaying_cloud_aggregator_node
  ${catkin_LIBRARIES}
)

install(TARGETS decaying_cloud_aggregator_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


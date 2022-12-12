cmake_minimum_required(VERSION 3.0.2)
project(arducam_v4l2_ros)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++14)
set(CMAKE_CXX_FLAGS_RELEASE "-g -O3 -Wno-write-strings -Wno-format-overflow")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  nodelet
  roscpp
  sensor_msgs
)

#Find opencv
find_package(OpenCV 4.5 REQUIRED )
# find_package()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES arducam_ros
#  CATKIN_DEPENDS cv_bridge image_transport nodelet roscpp sensor_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_library(arducam_driver_v4l2
  src/arducam_driver_v4l2.cpp
)

add_dependencies(arducam_driver_v4l2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/arducam_v4l2_ros_node.cpp)

add_dependencies(${PROJECT_NAME}_node arducam_driver_v4l2 ${catkin_EXPORTED_TARGETS})

target_link_libraries(arducam_driver_v4l2
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_node
  arducam_driver_v4l2
  ${catkin_LIBRARIES}
)

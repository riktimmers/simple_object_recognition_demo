cmake_minimum_required(VERSION 3.0.2)
project(perception)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE=Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  pcl_ros
  tf2_ros
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Area.msg
  BoundingBox.msg
  FilterOptions.msg
  ROI.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  FindBoundingBox.srv
  FindROIs.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs 
)

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/perception_node.cpp
                                    src/perception.cpp
                                    src/perception_service.cpp
                                    src/perception_find_bounding_box.cpp
                                    src/perception_point_cloud_function.cpp
                                    src/perception_find_rois.cpp
)

add_dependencies(${PROJECT_NAME}_node perception_generate_messages_cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

cmake_minimum_required(VERSION 3.5)
project(lidar_tools)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)

# pcl
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

# find Boost
find_package(Boost COMPONENTS program_options filesystem REQUIRED)

if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
else()
  message(FATAL_ERROR "Could not locate Boost")
endif()

# ros
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  sensor_msgs
  tf2
  message_filters
  visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp roslib std_msgs sensor_msgs tf2 visualization_msgs
  DEPENDS OpenCV PCL
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

aux_source_directory(./src/merge_pc SRC0)
add_executable(merge_pc ${SRC0})
target_link_libraries(merge_pc
  yaml-cpp
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_dependencies(merge_pc ${catkin_EXPORTED_TARGETS})

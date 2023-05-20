cmake_minimum_required(VERSION 3.1)
project(nclt2bag)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
message(STATUS "Current build type: ${CMAKE_BUILD_TYPE}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rosbag
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(LibArchive REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(nclt2bag src/main.cpp )
target_link_libraries(nclt2bag ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${LibArchive_LIBRARIES})

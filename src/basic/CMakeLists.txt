cmake_minimum_required(VERSION 3.16)
project(basic LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -pthread -fexceptions")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(glog REQUIRED)
find_package(gflags REQUIRED)

# Conda/modern glog needs this when consuming glog headers from legacy CMake projects.
add_compile_definitions(GLOG_USE_GLOG_EXPORT)

if(TARGET gflags::gflags)
  set(GFLAGS_TARGET gflags::gflags)
else()
  set(GFLAGS_TARGET gflags)
endif()

catkin_package(
  CATKIN_DEPENDS roscpp rospy
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS include
  LIBRARIES basic
)

file(GLOB_RECURSE BASIC_SOURCES
  "src/*.cpp"
)

file(GLOB_RECURSE BASIC_HEADERS
  "include/*.h"
  "include/*.hpp"
)

add_library(basic
  ${BASIC_SOURCES}
  ${BASIC_HEADERS}
)

target_include_directories(basic PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

target_compile_definitions(basic PUBLIC
  GLOG_USE_GLOG_EXPORT
)

target_link_libraries(basic PUBLIC
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  glog::glog
  ${GFLAGS_TARGET}
)
cmake_minimum_required(VERSION 3.16)
project(super_lio LANGUAGES CXX)

# DEBUG CONFIGURATION
# set(CMAKE_BUILD_TYPE Debug)
# set(CMAKE_CXX_FLAGS_DEBUG "-O0 -Wall -g -pthread -fsanitize=address")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -pthread -fexceptions -flto=auto")

# Needed by project source code.
add_compile_definitions(ROOT="${CMAKE_CURRENT_SOURCE_DIR}/")

# Conda/modern glog needs this when consuming glog headers from legacy CMake projects.
add_compile_definitions(GLOG_USE_GLOG_EXPORT)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(glog REQUIRED)
find_package(gflags REQUIRED)
find_package(TBB REQUIRED)
find_package(PythonLibs REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  basic
  pcl_ros
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  nav_msgs
  message_generation
)

if(TARGET gflags::gflags)
  set(GFLAGS_TARGET gflags::gflags)
else()
  set(GFLAGS_TARGET gflags)
endif()

if(TARGET TBB::tbb)
  set(TBB_TARGET TBB::tbb)
else()
  set(TBB_TARGET tbb)
endif()

add_message_files(
  FILES
  CloudPose.msg
  CloudPose2.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    basic
    pcl_ros
    geometry_msgs
    nav_msgs
    roscpp
    rospy
    std_msgs
    sensor_msgs
    visualization_msgs
    message_runtime
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS include
  LIBRARIES lio
)

file(GLOB_RECURSE LIO_LIB_SRC
  "src/lio/*.cpp"
  "src/ros/*.cpp"
)

file(GLOB_RECURSE LIO_LIB_HEADERS
  "include/lio/*.hpp"
  "include/lio/*.h"
)

add_library(lio
  ${LIO_LIB_SRC}
  ${LIO_LIB_HEADERS}
)

add_dependencies(lio
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_include_directories(lio PUBLIC
  include
  3rdparty
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
)

target_compile_definitions(lio PUBLIC
  GLOG_USE_GLOG_EXPORT
  ROOT="${CMAKE_CURRENT_SOURCE_DIR}/"
)

target_link_libraries(lio PUBLIC
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${PYTHON_LIBRARIES}
  glog::glog
  ${GFLAGS_TARGET}
  ${TBB_TARGET}
)

add_executable(super_lio_node
  src/apps/super_lio_node.cpp
)

add_dependencies(super_lio_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(super_lio_node
  lio
  ${catkin_LIBRARIES}
)

add_executable(relocation_node
  src/apps/relocation_node.cpp
)

add_dependencies(relocation_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(relocation_node
  lio
  ${catkin_LIBRARIES}
)
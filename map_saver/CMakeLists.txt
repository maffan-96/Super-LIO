cmake_minimum_required(VERSION 3.8)
project(map_saver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(Eigen3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Per-frame registered-cloud + pose saver (FAST-LIO-style, frame-aware)
add_executable(registered_cloud_pose_saver src/pose_saver.cpp)
target_link_libraries(registered_cloud_pose_saver ${PCL_LIBRARIES} Eigen3::Eigen)
ament_target_dependencies(registered_cloud_pose_saver
  rclcpp sensor_msgs nav_msgs pcl_conversions)

# Install rule so ROS 2 can find the executable
install(TARGETS
  registered_cloud_pose_saver
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ament_package() must be the LAST call so targets/installs above are registered.
ament_package()

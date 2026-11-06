cmake_minimum_required(VERSION 3.20)
project(adaptive_costmap_2d VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

if(EXISTS "/opt/ros/jazzy/include/karto_sdk")
  include_directories(/opt/ros/jazzy/include/karto_sdk)
endif()

# Core Shared Library (Nav2 Costmap Layer Plugin)
add_library(adaptive_costmap_2d_core SHARED
  src/adaptive_costmap_layer.cpp
)

target_include_directories(adaptive_costmap_2d_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(adaptive_costmap_2d_core
  rclcpp
  nav2_costmap_2d
  nav2_util
  pluginlib
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
)

pluginlib_export_plugin_description_file(nav2_costmap_2d plugins/adaptive_costmap_plugins.xml)

# Standalone Empirical Benchmark Executable
add_executable(benchmark_memory_and_speed
  benchmark/benchmark_memory_and_speed.cpp
)

target_include_directories(benchmark_memory_and_speed PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(benchmark_memory_and_speed
  nav_msgs
)

# Standalone Adaptive Mapping Node
add_executable(adaptive_mapping_node
  src/adaptive_mapping_node.cpp
)

target_include_directories(adaptive_mapping_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(adaptive_mapping_node
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(GTest REQUIRED)

  ament_add_gtest(test_sparse_grid
    test/test_sparse_grid.cpp
  )

  target_include_directories(test_sparse_grid PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  target_link_libraries(test_sparse_grid
    GTest::gtest
    GTest::gtest_main
  )
  ament_target_dependencies(test_sparse_grid
    rclcpp
    geometry_msgs
    nav_msgs
    visualization_msgs
  )

  ament_add_gtest(test_plugin_loading
    test/test_plugin_loading.cpp
  )

  target_include_directories(test_plugin_loading PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  target_link_libraries(test_plugin_loading
    GTest::gtest
    GTest::gtest_main
    pluginlib::pluginlib
  )
  ament_target_dependencies(test_plugin_loading
    nav2_costmap_2d
    pluginlib
  )
endif()

# Installation
install(TARGETS
  adaptive_costmap_2d_core
  benchmark_memory_and_speed
  adaptive_mapping_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES plugins/adaptive_costmap_plugins.xml
  DESTINATION share/${PROJECT_NAME}/plugins
)

ament_export_include_directories(include)
ament_export_libraries(adaptive_costmap_2d_core)
ament_export_dependencies(
  rclcpp
  nav2_costmap_2d
  nav2_util
  pluginlib
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
)

ament_package()

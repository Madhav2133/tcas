cmake_minimum_required(VERSION 3.5)
project(tcas)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(dependencies
  "rclcpp"
  "std_msgs"
  "nav_msgs"
  "geometry_msgs"
  "lifecycle_msgs"
  "nav2_msgs"
  "sensor_msgs"
)

add_library(
  ${PROJECT_NAME} SHARED
  src/JunctionMonitor.cc
  include/JunctionMonitor.h
  include/CustomTypes.h)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include)

#To run vio system
add_executable(run_tcas run_tcas.cpp src/JunctionMonitor.cc)
ament_target_dependencies(run_tcas rclcpp nav_msgs geometry_msgs lifecycle_msgs nav2_msgs sensor_msgs std_msgs)

add_executable(lane_modifier lane_modifier.cpp)
ament_target_dependencies(lane_modifier rclcpp geometry_msgs nav_msgs nav2_msgs sensor_msgs std_msgs lifecycle_msgs)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

target_link_libraries(run_tcas ${PROJECT_NAME})
target_link_libraries(lane_modifier ${PROJECT_NAME})

install(TARGETS
   run_tcas lane_modifier
  DESTINATION lib/${PROJECT_NAME})

# ament_export_dependencies(nav2_bt_navigator)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(lifecycle_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(nav2_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(rclcpp)
ament_package()
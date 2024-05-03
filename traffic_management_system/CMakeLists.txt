cmake_minimum_required(VERSION 3.5)
project(traffic_management_system)

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
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# find_package(traffic_management_system REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(dependencies
  "rclcpp"
  "nav_msgs"
  "nav2_msgs"
  "geometry_msgs"
  "sensor_msgs"
)

# add_executable(junction_details src/junctionDetails.cpp)
# ament_target_dependencies(junction_details rclcpp)

# add_executable(junction_monitor src/junctionMonitor.cpp)
# ament_target_dependencies(junction_monitor rclcpp nav_msgs geometry_msgs traffic_management_system)

add_executable(plan_modifier src/testNode.cpp)
ament_target_dependencies(plan_modifier rclcpp nav_msgs geometry_msgs nav2_msgs sensor_msgs)

# add_executable(robot_planner src/robotPlanner.cpp)
# ament_target_dependencies(robot_planner rclcpp geometry_msgs traffic_management_system)

# find_package(rosidl_default_generators REQUIRED)

# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/RobotDetails.msg"
# )

install(TARGETS
   plan_modifier 
  DESTINATION lib/${PROJECT_NAME})

# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}
# )
# install(DIRECTORY
#   config
#   DESTINATION share/${PROJECT_NAME}
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(nav2_msgs)
ament_export_dependencies(rclcpp)
ament_package()



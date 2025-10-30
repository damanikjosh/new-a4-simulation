cmake_minimum_required(VERSION 3.8)
project(mission_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mission_planner_msgs REQUIRED)
find_package(MAVSDK REQUIRED)
find_package(rclpy REQUIRED)


include_directories(
  data include
)

# Optional: Obstacle-aware trajectory generation
# Uncomment to enable obstacle avoidance features
# Requires additional dependencies: GDAL, CGAL or Boost.Geometry
# 
# find_package(GDAL REQUIRED)
# find_package(CGAL REQUIRED)
# set(OBSTACLE_AVOIDANCE_SOURCES src/obstacle_aware_trajectory_generator.cpp)
# set(OBSTACLE_AVOIDANCE_LIBS ${GDAL_LIBRARIES} CGAL::CGAL)

add_executable(vehicle 
  src/vehicle.cpp
  src/trajectory_generator.cpp
  src/actions/goto_action.cpp
  src/actions/rtl_action.cpp
  src/actions/waypoints_action.cpp
  src/actions/followme_action.cpp
  src/actions/search_action.cpp
  # ${OBSTACLE_AVOIDANCE_SOURCES}  # Uncomment when enabling obstacle avoidance
)
ament_target_dependencies(vehicle rclcpp rclcpp_action std_msgs sensor_msgs mission_planner_msgs)
target_link_libraries(vehicle MAVSDK::mavsdk)  # ${OBSTACLE_AVOIDANCE_LIBS}  # Uncomment when enabling

install(TARGETS vehicle
  DESTINATION lib/${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  scripts/mission_planner_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install data and launch files
install(DIRECTORY data launch
  DESTINATION share/${PROJECT_NAME}
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

ament_package()

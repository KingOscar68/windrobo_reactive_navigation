cmake_minimum_required(VERSION 3.8)
project(windrobo_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(windrobo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


# Create executable
add_executable(goal_subscriber src/goal_subscriber.cpp)

ament_target_dependencies(goal_subscriber
  rclcpp
  windrobo_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
)

install(TARGETS
  goal_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()


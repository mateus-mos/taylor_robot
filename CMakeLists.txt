cmake_minimum_required(VERSION 3.5)
project(taylor_robot)

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
find_package(ros2_control REQUIRED)
find_package(robot_localization REQUIRED)
find_package(nav2_core REQUIRED)
find_package(gazebo_ros)

install(
	DIRECTORY config launch description 
	DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/config/env-hooks/${PROJECT_NAME}.sh.in")

ament_package()
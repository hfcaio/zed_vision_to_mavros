cmake_minimum_required(VERSION 3.8)
project(zed_vision_to_mavros)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic )
endif()


# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Include Cpp "include" directory
#include_directories(include)

# Create Cpp executable
add_executable(zed_vision_to_mavros src/zed_vision_to_mavros.cpp)
ament_target_dependencies(zed_vision_to_mavros rclcpp geometry_msgs tf2 tf2_ros)

# Install Cpp executables
install(TARGETS
  zed_vision_to_mavros
  DESTINATION lib/${PROJECT_NAME}
)


# Launch files
#install(TARGETS
#  launch
#  DESTINATION lib/${PROJECT_NAME}
#)


# Install Python modules
#ament_python_install_package(${PROJECT_NAME})


# Install Python executables
#install(PROGRAMS
#  scripts/main.py
#  DESTINATION lib/${PROJECT_NAME}
#)

ament_package()

cmake_minimum_required(VERSION 3.5)
project(imu)

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

find_package(serial REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp  REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  serial
  sensor_msgs
  std_msgs
  ament_cmake
  rclcpp
)

# Include directories
include_directories(
  include
  
  ${ament_INCLUDE_DIRS}
  ${THIS_PACKAGE_INCLUDE_DIRS}
 
)

# Declare dependencies
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_include_directories(include)

add_executable(imu_node src/imu_node.cpp src/imu_cmd.cpp)
target_include_directories(imu_node PUBLIC include)
ament_target_dependencies(imu_node ${THIS_PACKAGE_INCLUDE_DEPENDS})

set_target_properties(imu_node PROPERTIES INSTALL_RPATH "/usr/local/lib")


install(TARGETS
  imu_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})
  
ament_package()
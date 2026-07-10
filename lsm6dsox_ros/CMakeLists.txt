cmake_minimum_required(VERSION 3.8)
project(lsm6dsox_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Threads REQUIRED)

add_executable(imu_publisher src/imu_publisher.cpp)
ament_target_dependencies(imu_publisher rclcpp sensor_msgs)
target_link_libraries(imu_publisher Threads::Threads)

install(TARGETS
  imu_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

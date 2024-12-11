cmake_minimum_required(VERSION 3.8)
project(arm_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# If tmotor is installed system-wide and you have /usr/local paths correct
include_directories(/usr/local/include)
link_directories(/usr/local/lib)

add_executable(arm_controller src/arm_controller.cpp)

# Use ament_target_dependencies for rclcpp and sensor_msgs
ament_target_dependencies(arm_controller rclcpp sensor_msgs)

# Link only non-ROS libraries manually if needed
target_link_libraries(arm_controller tmotor)

install(TARGETS
  arm_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

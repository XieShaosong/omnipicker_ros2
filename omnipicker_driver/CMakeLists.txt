cmake_minimum_required(VERSION 3.8)
project(omnipicker_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(omnipicker_interfaces REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
)

add_executable(omnipicker_driver src/omnipicker_driver.cpp src/serial_port.cpp src/parse_data.cpp)
ament_target_dependencies(omnipicker_driver
  rclcpp
  std_msgs
  omnipicker_interfaces
)

install(TARGETS omnipicker_driver
  DESTINATION lib/${PROJECT_NAME})

ament_package()

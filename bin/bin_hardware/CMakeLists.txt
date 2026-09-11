cmake_minimum_required(VERSION 3.8)
project(bin_hardware)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED libserial)

add_library(bin_hardware SHARED
  src/diffbot_system.cpp
)

target_include_directories(bin_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${LIBSERIAL_INCLUDE_DIRS}
)

ament_target_dependencies(bin_hardware
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  std_msgs
)

# MUST come AFTER ament_target_dependencies — libserial ordering matters
target_link_libraries(bin_hardware
  ${LIBSERIAL_LIBRARIES}
)

pluginlib_export_plugin_description_file(
  hardware_interface
  bin_hardware.xml
)

install(
  TARGETS bin_hardware
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES bin_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

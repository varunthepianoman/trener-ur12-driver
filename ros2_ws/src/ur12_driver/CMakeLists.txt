cmake_minimum_required(VERSION 3.8)
project(ur12_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ur12_driver_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# ------------------------------------------------------------------
# Driver node executable
# ------------------------------------------------------------------
add_executable(ur_driver_node
  src/ur_driver_node.cpp
  src/main.cpp
)

target_include_directories(ur_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(ur_driver_node
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  sensor_msgs
  std_srvs
  ur12_driver_msgs
)

# ------------------------------------------------------------------
# send_burst — fires N MoveJoints goals at fixed gaps from one node
# ------------------------------------------------------------------
add_executable(send_burst src/send_burst.cpp)
ament_target_dependencies(send_burst
  rclcpp
  rclcpp_action
  sensor_msgs
  ur12_driver_msgs
)

add_executable(cancel_goal src/cancel_goal.cpp)
ament_target_dependencies(cancel_goal
  rclcpp
  rclcpp_action
  sensor_msgs
  ur12_driver_msgs
)

# ------------------------------------------------------------------
# Hardware interface shared library (pluginlib plugin)
# ------------------------------------------------------------------
add_library(ur12_hardware_interface SHARED
  src/ur12_hardware_interface.cpp
)

target_include_directories(ur12_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(ur12_hardware_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface ur12_hardware_interface.xml)

# ------------------------------------------------------------------
# Install
# ------------------------------------------------------------------
install(TARGETS ur_driver_node send_burst cancel_goal ur12_hardware_interface
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES ur12_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

# ------------------------------------------------------------------
# Tests
# ------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_dashboard_client test/test_dashboard_client.cpp)
  target_include_directories(test_dashboard_client PRIVATE include)

  ament_add_gtest(test_rtde_client test/test_rtde_client.cpp)
  target_include_directories(test_rtde_client PRIVATE include)
  ament_target_dependencies(test_rtde_client rclcpp)

  ament_add_gtest(test_script_client test/test_script_client.cpp)
  target_include_directories(test_script_client PRIVATE include)
  ament_target_dependencies(test_script_client rclcpp)
endif()

ament_package()

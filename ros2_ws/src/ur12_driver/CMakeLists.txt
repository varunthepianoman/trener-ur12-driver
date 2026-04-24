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
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ur12_driver_msgs REQUIRED)

# Driver node executable
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
  sensor_msgs
  std_srvs
  ur12_driver_msgs
)

# Install
install(TARGETS ur_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# Tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_dashboard_client test/test_dashboard_client.cpp)
  target_include_directories(test_dashboard_client PRIVATE include)

  ament_add_gtest(test_rtde_client test/test_rtde_client.cpp)
  target_include_directories(test_rtde_client PRIVATE include)
endif()

ament_package()

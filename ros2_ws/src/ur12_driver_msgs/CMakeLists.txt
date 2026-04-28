cmake_minimum_required(VERSION 3.8)
project(ur12_driver_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveFirstJoint.srv"
  "action/MoveHome.action"
  "action/MoveFirstJoint.action"
  "action/MoveJoints.action"
  DEPENDENCIES action_msgs
)

ament_package()

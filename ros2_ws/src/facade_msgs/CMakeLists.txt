cmake_minimum_required(VERSION 3.8)
project(facade_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ReadJointPositions.srv"
  "srv/MoveToPose.srv"
  "srv/ReadToolPose.srv"
  "msg/Waypoint.msg"
  "action/FollowTrajectory.action"
  DEPENDENCIES action_msgs
)

ament_package()

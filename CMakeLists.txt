cmake_minimum_required(VERSION 3.0.2)
project(autonomous_nav)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  actionlib
  move_base_msgs
  vision_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    rospy 
    std_msgs 
    sensor_msgs 
    geometry_msgs 
    cv_bridge
    actionlib
    move_base_msgs
    vision_msgs
)

install(PROGRAMS
  scripts/yolo_detection_node.py
  scripts/obstacle_avoidance_node.py
  scripts/navigation_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  config
  models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
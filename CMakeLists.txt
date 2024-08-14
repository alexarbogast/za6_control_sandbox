cmake_minimum_required(VERSION 3.0.2)
project(za_control_sandbox)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

catkin_package(CATKIN_DEPENDS
  roscpp
  rospy
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS
  scripts/coordinated_motion_demo.py
  scripts/nullspace_control_demo.py
  scripts/twist_decomposition_demo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  config
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

cmake_minimum_required(VERSION 2.8.3)
project(qp_speed_replanner)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  autoware_msgs
  autoware_build_flags
  std_msgs
  tf
  python_qt_binding
  dynamic_reconfigure
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/qp_speed_replanner.cfg
)

catkin_package(CATKIN_DEPENDS
  roscpp
  rospy
  autoware_msgs
  autoware_build_flags
  std_msgs
  tf
  python_qt_binding
  dynamic_reconfigure
)

catkin_install_python(PROGRAMS scripts/qp_speed_replanner scripts/utils.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


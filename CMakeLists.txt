cmake_minimum_required(VERSION 2.8.3)
project(box_publisher)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  rospy
)
find_package(PythonLibs REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/DynamicParams.cfg
  cfg/PolygonArraytoPolygonConfig.cfg
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
)

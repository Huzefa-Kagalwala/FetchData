cmake_minimum_required(VERSION 2.8.3)
project(fetch_description)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
)

catkin_package()

install(DIRECTORY cmake meshes robots
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

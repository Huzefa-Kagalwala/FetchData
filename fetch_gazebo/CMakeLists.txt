cmake_minimum_required(VERSION 2.8.3)
project(fetch_gazebo)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
)

catkin_package()

install(DIRECTORY cmake include launch models robots worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

cmake_minimum_required(VERSION 2.8.3)
project( imu_odom )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
  tf
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS
    serial
    std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable( imu_odom
  src/main.cc
  src/crc.cc
  src/udp.cc
  src/serial.cc
)

target_link_libraries( imu_odom
  ${catkin_LIBRARIES}
)


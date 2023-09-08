cmake_minimum_required(VERSION 2.8.3)
project(self_drive_in_furrow)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(self_drive_in_furrow src/self_drive_in_furrow.cpp)

target_link_libraries(self_drive_in_furrow
  ${catkin_LIBRARIES}
)


cmake_minimum_required(VERSION 2.8.3)
project(imu_publish)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES imu_publish
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(imu src/main.cpp src/com.cpp src/atomprotocol.cpp)
target_link_libraries(imu ${catkin_LIBRARIES})

add_executable(receive src/receive.cpp)
target_link_libraries(receive ${catkin_LIBRARIES})

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(
  TARGETS
    imu
    receive
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
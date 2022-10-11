cmake_minimum_required(VERSION 3.0.2)
project(event_recording)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslaunch
  std_msgs
  geometry_msgs
  rosbag_snapshot_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs rosbag_snapshot_msgs
)

add_executable(snapshot_trigger_node src/snapshot_wrapper.cpp)
add_dependencies(snapshot_trigger_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(snapshot_trigger_node ${catkin_LIBRARIES})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(PROGRAMS scripts/prometheus_exporter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS snapshot_trigger_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  roslaunch_add_file_check(launch/prometheus_exporter.launch)
  roslaunch_add_file_check(launch/snapshot_buffer.launch)
endif()


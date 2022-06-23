cmake_minimum_required(VERSION 3.0.2)
project(tello_navi)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tello_navi
#  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  "${PROJECT_SOURCE_DIR}/include"
  "/usr/include/eigen3"
)


add_executable(joy_ope_node src/joy_ope_node.cpp src/formation_flight_utility.cpp)
target_link_libraries(joy_ope_node ${catkin_LIBRARIES})

add_executable(takeoff src/takeoff.cpp)
target_link_libraries(takeoff ${catkin_LIBRARIES})

add_executable(land src/land.cpp)
target_link_libraries(land ${catkin_LIBRARIES})

add_executable(simple_tello_sim src/simple_tello_sim.cpp)
target_link_libraries(simple_tello_sim ${catkin_LIBRARIES})

add_executable(lqr_hovering src/lqr_hovering.cpp)
target_link_libraries(lqr_hovering ${catkin_LIBRARIES})

add_executable(target_sender src/target_sender.cpp)
target_link_libraries(target_sender ${catkin_LIBRARIES})

add_executable(tf_markers src/tf_markers.cpp)
target_link_libraries(tf_markers ${catkin_LIBRARIES})

add_executable(formation_flight_lqr src/formation_flight_lqr.cpp src/formation_flight_utility.cpp)
target_link_libraries(formation_flight_lqr ${catkin_LIBRARIES})

add_executable(pose_dumper src/pose_dumper.cpp src/formation_flight_utility.cpp)
target_link_libraries(pose_dumper ${catkin_LIBRARIES})

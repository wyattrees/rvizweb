# toplevel CMakeLists.txt for a catkin workspace
# catkin/cmake/toplevel.cmake

cmake_minimum_required(VERSION 3.0.2)
project(traj_manager)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  trajectory_msgs
  control_msgs
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES torque_controller
  CATKIN_DEPENDS rospy std_msgs trajectory_msgs control_msgs sensor_msgs message_runtime
#  DEPENDS system_lib
)

catkin_python_setup()


catkin_install_python(PROGRAMS
    scripts/run_traj
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

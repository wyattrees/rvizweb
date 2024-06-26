cmake_minimum_required(VERSION 2.8.3)
project(rvizweb)

find_package(catkin REQUIRED COMPONENTS
  rosbridge_server
  roswww
  tf2_web_republisher
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Copy needed files to www build folder
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/www)
execute_process(
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/www
  COMMAND mkdir css
)
execute_process(
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/www
  COMMAND mkdir js
)
configure_file(package.json www/package.json COPYONLY)
configure_file(index.html www/index.html COPYONLY)
configure_file(bower.json www/bower.json COPYONLY)

configure_file(css/mediapipe.css www/css/mediapipe.css COPYONLY)
configure_file(css/wy.css www/css/wy.css COPYONLY)
configure_file(js/mediapipe.js www/js/mediapipe.js COPYONLY)
# Install webapp build tools
execute_process(
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/www
  COMMAND npm install
)

# Build the webapp with bower and polymer
execute_process(
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/www
  COMMAND npm run build
)

install(
  DIRECTORY
    launch
    ${CMAKE_CURRENT_BINARY_DIR}/www/build/www
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY
    config
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

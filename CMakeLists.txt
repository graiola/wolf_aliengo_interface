cmake_minimum_required(VERSION 3.0.2)
project(wolf_aliengo_interface)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  realtime_tools
  wb_hardware_interface
)
      
add_subdirectory(aliengo_hal)

catkin_package(
 CATKIN_DEPENDS
   controller_manager
   realtime_tools
   wb_hardware_interface
)

##########
# Build ##
##########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(aliengo_ros_node
    src/aliengo_robot_hw.cpp
    src/aliengo_ros_control.cpp
    src/aliengo_ros_node.cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(aliengo_ros_node
  ${catkin_LIBRARIES}
  aliengohal
)

set_property(TARGET aliengo_ros_node PROPERTY CXX_STANDARD 11)  # At least std c++11

#############
## Install ##
#############

install(
  TARGETS aliengo_ros_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


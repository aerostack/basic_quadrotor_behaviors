cmake_minimum_required(VERSION 2.8.3)
project(basic_quadrotor_behaviors)

### Use version 2011 of C++ (c++11). By default ROS uses c++98
#see: http://stackoverflow.com/questions/10851247/how-to-activate-c-11-in-cmake
#see: http://stackoverflow.com/questions/10984442/how-to-detect-c11-support-of-a-compiler-with-cmake
add_definitions(-std=c++11)

# Directories definition
set(BASIC_QUADROTOR_BEHAVIORS_SOURCE_DIR
  src/source
)

set(BASIC_QUADROTOR_BEHAVIORS_INCLUDE_DIR
  src/include
)

set(BASIC_QUADROTOR_BEHAVIORS_HEADER_FILES
        ${BASIC_QUADROTOR_BEHAVIORS_INCLUDE_DIR}/behavior_take_off.h
        ${BASIC_QUADROTOR_BEHAVIORS_INCLUDE_DIR}/behavior_land.h
        ${BASIC_QUADROTOR_BEHAVIORS_INCLUDE_DIR}/behavior_wait.h
        ${BASIC_QUADROTOR_BEHAVIORS_INCLUDE_DIR}/behavior_estimate_position_with_linear_speed.h
        ${BASIC_QUADROTOR_BEHAVIORS_INCLUDE_DIR}/behavior_estimate_position_with_sensor.h
        ${BASIC_QUADROTOR_BEHAVIORS_INCLUDE_DIR}/behavior_self_localize_with_ground_truth.h
        ${BASIC_QUADROTOR_BEHAVIORS_INCLUDE_DIR}/behavior_hover_with_flight_action_control.h
)

set(BASIC_QUADROTOR_BEHAVIORS_SOURCE_FILES
        ${BASIC_QUADROTOR_BEHAVIORS_SOURCE_DIR}/behavior_take_off.cpp
        ${BASIC_QUADROTOR_BEHAVIORS_SOURCE_DIR}/behavior_land.cpp
        ${BASIC_QUADROTOR_BEHAVIORS_SOURCE_DIR}/behavior_wait.cpp
        ${BASIC_QUADROTOR_BEHAVIORS_SOURCE_DIR}/behavior_estimate_position_with_linear_speed.cpp
        ${BASIC_QUADROTOR_BEHAVIORS_SOURCE_DIR}/behavior_estimate_position_with_sensor.cpp
        ${BASIC_QUADROTOR_BEHAVIORS_SOURCE_DIR}/behavior_self_localize_with_ground_truth.cpp
        ${BASIC_QUADROTOR_BEHAVIORS_SOURCE_DIR}/behavior_hover_with_flight_action_control.cpp
)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  behavior_execution_manager
  geometry_msgs
  aerostack_msgs
  behavior_execution_manager_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS ${BASIC_QUADROTOR_BEHAVIORS_INCLUDE_DIR}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
  roscpp
  std_msgs
  behavior_execution_manager
  geometry_msgs
  aerostack_msgs
  behavior_execution_manager_msgs
  DEPENDS yaml-cpp
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(BehaviorTakeOff ${BASIC_QUADROTOR_BEHAVIORS_SOURCE_DIR}/behavior_take_off.cpp)
target_link_libraries(BehaviorTakeOff ${catkin_LIBRARIES})
target_link_libraries(BehaviorTakeOff yaml-cpp)
add_executable(BehaviorLand ${BASIC_QUADROTOR_BEHAVIORS_SOURCE_DIR}/behavior_land.cpp)
target_link_libraries(BehaviorLand ${catkin_LIBRARIES})
target_link_libraries(BehaviorLand yaml-cpp)
add_executable(BehaviorWait ${BASIC_QUADROTOR_BEHAVIORS_SOURCE_DIR}/behavior_wait.cpp)
target_link_libraries(BehaviorWait ${catkin_LIBRARIES})
target_link_libraries(BehaviorWait yaml-cpp)
add_executable(BehaviorEstimatePositionWithLinearSpeed ${BASIC_QUADROTOR_BEHAVIORS_SOURCE_DIR}/behavior_estimate_position_with_linear_speed.cpp)
target_link_libraries(BehaviorEstimatePositionWithLinearSpeed ${catkin_LIBRARIES})
target_link_libraries(BehaviorEstimatePositionWithLinearSpeed yaml-cpp)
add_executable(BehaviorEstimatePositionWithSensor ${BASIC_QUADROTOR_BEHAVIORS_SOURCE_DIR}/behavior_estimate_position_with_sensor.cpp)
target_link_libraries(BehaviorEstimatePositionWithSensor ${catkin_LIBRARIES})
target_link_libraries(BehaviorEstimatePositionWithSensor yaml-cpp)
add_executable(BehaviorSelfLocalizeWithGroundTruth ${BASIC_QUADROTOR_BEHAVIORS_SOURCE_DIR}/behavior_self_localize_with_ground_truth.cpp)
target_link_libraries(BehaviorSelfLocalizeWithGroundTruth ${catkin_LIBRARIES})
target_link_libraries(BehaviorSelfLocalizeWithGroundTruth yaml-cpp)
add_executable(BehaviorHoverWithFlightActionControl ${BASIC_QUADROTOR_BEHAVIORS_SOURCE_DIR}/behavior_hover_with_flight_action_control.cpp)
target_link_libraries(BehaviorHoverWithFlightActionControl ${catkin_LIBRARIES})
target_link_libraries(BehaviorHoverWithFlightActionControl yaml-cpp)

if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(BehaviorTakeOff ${catkin_EXPORTED_LIBRARIES})
  add_dependencies(BehaviorLand ${catkin_EXPORTED_LIBRARIES})
  add_dependencies(BehaviorWait ${catkin_EXPORTED_LIBRARIES})
  add_dependencies(BehaviorEstimatePositionWithLinearSpeed ${catkin_EXPORTED_LIBRARIES})
  add_dependencies(BehaviorEstimatePositionWithSensor ${catkin_EXPORTED_LIBRARIES})
  add_dependencies(BehaviorSelfLocalizeWithGroundTruth ${catkin_EXPORTED_LIBRARIES})
  add_dependencies(BehaviorHoverWithFlightActionControl ${catkin_EXPORTED_LIBRARIES})
endif()

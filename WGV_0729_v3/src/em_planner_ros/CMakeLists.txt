cmake_minimum_required(VERSION 3.0.2)
project(em_planner_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)
find_package(Python3 3.8 REQUIRED COMPONENTS Interpreter Development NumPy)
set(EIGEN_LIB "common/eigen3")
set(OSQP_LIB "common/osqp")


include_directories(
  include
  ${EIGEN_LIB}
  common
  emplanner/include
  scenario/include
  ${OSQP_LIB}/include
  ${catkin_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
  ${Python3_NumPy_INCLUDE_DIRS}
)



## Generate added messages and services with any dependencies listed here
add_message_files(
  FILES
  FrenetPoint.msg
  TrajectoryPoint.msg
  PlanningResult.msg
  Obstacle.msg
  ObstacleList.msg
  Localization.msg
  VehicleStatus.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)


###################################
## catkin specific configuration ##
###################################

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

###########
## Build ##
###########

add_executable(${PROJECT_NAME}_node
    src/main2.cpp
    scenario/src/sce1.cpp
    scenario/src/sce2.cpp
    scenario/src/sce3.cpp
    scenario/src/sce4.cpp
    common/cartesian_frenet_conversion.cc
    common/discrete_points_math.cc
    emplanner/src/em_planner.cpp
    emplanner/src/sl_dp.cpp
    emplanner/src/sl_qp.cpp
    emplanner/src/st_dp.cpp
    emplanner/src/st_qp.cpp
)

add_executable(environment_publisher_node
    src/EnvironmentPublisher.cpp
    scenario/src/sce1.cpp
    scenario/src/sce2.cpp
    scenario/src/sce3.cpp
    common/cartesian_frenet_conversion.cc
    common/discrete_points_math.cc
)


## Specify libraries to link a library or executable target against
target_link_directories(${PROJECT_NAME}_node PRIVATE
    ${OSQP_LIB}/lib
)
target_link_libraries(${PROJECT_NAME}_node
  osqp
  ${catkin_LIBRARIES}
  Python3::Python
  Python3::NumPy
)

target_link_libraries(environment_publisher_node
  ${catkin_LIBRARIES}
  Python3::Python
  Python3::NumPy
)


#############
## Install ##
#############

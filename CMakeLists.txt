cmake_minimum_required(VERSION 3.0.2)
project(airbot_play_test)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set(THREADS_PREFER_PTHREAD_FLAG ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  kdl_parser
  message_generation
  message_runtime
  robot_state_publisher
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

# find_package(mujoco REQUIRED)
find_package(ASSIMP REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(FCL REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(Threads)
find_package(orocos_kdl REQUIRED)
find_package(ompl REQUIRED)
find_package(ruckig REQUIRED)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS src
 LIBRARIES airbot_play_test
 CATKIN_DEPENDS kdl_parser message_generation message_runtime roscpp rospy sensor_msgs std_msgs tf
 DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  src
  ${catkin_INCLUDE_DIRS}
)
include_directories(
  ${EIGEN3_INCLUDE_DIRS}
)
include_directories(
  ${OMPL_INCLUDE_DIRS}
)
include_directories(
  ${FCL_INCLUDE_DIRS}
)
include_directories(
  src/libraries/moveit2_types
  src/libraries/ros2_msgs
  src/libraries/ros2_libs
)

## Declare a C++ library
# add_library(${PROJECT_NAME}_mujoco
#   src/modules/mujoco/simulator/glfw_dispatch.cc
#   src/modules/mujoco/simulator/simulate.cc
#   src/modules/mujoco/simulator/uitools.cc
#   src/modules/mujoco/mujocoInterface.cpp
#   src/modules/mujoco/mujocoSupport.cpp
# )

# target_compile_options(${PROJECT_NAME}_mujoco PUBLIC -DEIGEN_USE_BLAS)

# target_link_libraries(${PROJECT_NAME}_mujoco PUBLIC
#   ${catkin_LIBRARIES}
#   mujoco::mujoco
#   lodepng
#   glfw
#   openblas
# )

AUX_SOURCE_DIRECTORY(src/drivers/libcan PROJECT_LIBCAN_SOURCES)

add_library(${PROJECT_NAME}_drivers
  src/drivers/can.cpp
  ${PROJECT_LIBCAN_SOURCES}
)

target_link_libraries(${PROJECT_NAME}_drivers
  ${CMAKE_THREAD_LIBS_INIT}
)

file(GLOB MODULE_SOURCES
  "src/modules/math/*.cpp"
  "src/modules/collision_detection/*.cpp"
  "src/modules/control_algorithm/*.cpp"
  "src/modules/controller/*.cpp"
  "src/modules/motors/protocol/*.cpp"
  "src/modules/motors/*.cpp"
  "src/modules/state_machine/*.cpp"
  "src/modules/command/*.cpp"
  "src/modules/planners/ompl_planner/*.cpp"
  "src/modules/planners/planning_request_adapter/*.cpp"
  "src/modules/planners/trajectory_processing/*.cpp"
  "src/modules/kinematic_constraints/*.cpp"
)

add_library(${PROJECT_NAME}_modules ${MODULE_SOURCES})

target_link_libraries(${PROJECT_NAME}_modules
  ${PROJECT_NAME}_drivers
  ${ASSIMP_LIBRARIES}
  Eigen3::Eigen
  ${FCL_LIBRARIES}
  orocos-kdl
  ${OMPL_LIBRARIES}
  ruckig::ruckig
  kdl_parser
  ${Boost_LIBRARIES}
)

file(GLOB TASKS_SOURCES
  "src/tasks/*.cpp"
)

add_library(${PROJECT_NAME}_tasks
  ${TASKS_SOURCES}
  src/tasks/play_group_interface/play_group_interface.cpp
)

target_link_libraries(${PROJECT_NAME}_tasks
  ${PROJECT_NAME}_drivers
  ${PROJECT_NAME}_modules
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/arch/main.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_drivers
  ${PROJECT_NAME}_modules
  ${PROJECT_NAME}_tasks
)

add_executable(${PROJECT_NAME}_test test.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_drivers
  ${PROJECT_NAME}_modules
  ${PROJECT_NAME}_tasks
)

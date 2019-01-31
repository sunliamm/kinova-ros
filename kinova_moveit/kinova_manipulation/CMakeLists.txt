cmake_minimum_required(VERSION 2.8.3)
project(kinova_manipulation)

set (CMAKE_CXX_STANDARD 11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
             COMPONENTS
             moveit_core
             moveit_ros_planning
             moveit_ros_planning_interface
             pluginlib
             cmake_modules
             geometric_shapes
             message_generation
)

find_package(Boost REQUIRED system filesystem date_time thread)

find_package(Eigen3 REQUIRED)

find_package(kinova_driver REQUIRED)

add_service_files(
  FILES
  Pick_Place.srv
)

generate_messages(
  DEPENDENCIES 
  geometry_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  moveit_core
  moveit_ros_planning_interface
  interactive_markers
  message_runtime
)


###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

include_directories(${kinova_driver_INCLUDE_DIRS})


add_library(manipulation_lib src/manipulation.cpp include/manipulation.h)
target_link_libraries(manipulation_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${kinova_driver_LIBRARIES})
install(TARGETS manipulation_lib DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(pick_test src/pick_test.cpp)
target_link_libraries(pick_test manipulation_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${kinova_driver_LIBRARIES})
install(TARGETS pick_test DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(pick_server src/pick_server.cpp)
target_link_libraries(pick_server manipulation_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${kinova_driver_LIBRARIES})
if(catkin_EXPORTED_TARGETS)
  add_dependencies(pick_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
endif()
install(TARGETS pick_server DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})




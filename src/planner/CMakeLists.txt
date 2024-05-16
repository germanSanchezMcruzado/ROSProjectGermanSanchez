cmake_minimum_required(VERSION 2.8.3)
project(planner)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
)

## Find Eigen3 library
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

## Declare a catkin package
catkin_package()

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  include  # Add this line to include your header files directory
)

## Declare a C++ executable
add_executable(planner_node src/main.cpp src/Planner.cpp src/Node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(planner_node
  ${catkin_LIBRARIES}
)

install(TARGETS planner_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

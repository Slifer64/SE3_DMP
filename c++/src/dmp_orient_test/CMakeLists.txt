cmake_minimum_required(VERSION 2.8.3)

project(dmp_orient_test)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  dmp_lib
  io_lib
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS
)

include_directories(
  include
  /usr/include/eigen3
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/dmp_orient_test.cpp
  src/main.cpp
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  armadillo
  dmp_lib
  io_lib
)

cmake_minimum_required(VERSION 2.8.3)
project(veh_dyn_observer)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
  include
  CATKIN_DEPENDS
)

#############
## PID Lib ##
#############

add_library(veh_dyn_observer_api
  src/veh_dyn_observer.cpp
)

target_link_libraries(veh_dyn_observer_api
  ${EIGEN3_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_include_directories(veh_dyn_observer_api
  SYSTEM PUBLIC
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_dependencies(veh_dyn_observer_api
  ${catkin_EXPORTED_TARGETS}
)


#############
## Testing ##
#############

catkin_add_gtest(veh_dyn_observer_test test/veh_dyn_observer_test.cpp 
                WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/test/data/")

target_include_directories(veh_dyn_observer_test
  SYSTEM PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(veh_dyn_observer_test veh_dyn_observer_api 
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES})

add_dependencies(veh_dyn_observer_test
  ${catkin_EXPORTED_TARGETS})

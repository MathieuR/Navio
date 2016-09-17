cmake_minimum_required(VERSION 2.8.3)
project(navio)

# C++ 11
#set(CMAKE_CXX_FLAGS "-std=c++11 -Wall ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs sensor_msgs
)

###########
## Build ##
###########

include_directories(
  C++
  ${catkin_INCLUDE_DIRS}
)

# mpu9250 library
add_library(navio_mpu9250 STATIC C++/Navio/MPU9250.cpp)
#target_link_libraries(navio_mpu9250
#  ${catkin_LIBRARIES}
#)


# AHRS executable
add_executable(navio_ahrs src/AHRS.cpp)
#set_target_properties(navio_ahrs PROPERTIES OUTPUT_NAME ahrs PREFIX "")
target_link_libraries(navio_ahrs
  ${catkin_LIBRARIES}
  navio_mpu9250
)

install(TARGETS navio_ahrs
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
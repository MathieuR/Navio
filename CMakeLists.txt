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
add_library(${PROJECT_NAME}_mpu9250 C++/Navio/MPU9250.cpp)
target_link_libraries(navio_mpu9250
  ${catkin_LIBRARIES}
)

# gpio library
add_library(${PROJECT_NAME}_gpio C++/Navio/gpio.cpp)
target_link_libraries(${PROJECT_NAME}_gpio
  ${catkin_LIBRARIES}
)

# util library
add_library(${PROJECT_NAME}_util C++/Navio/Util.cpp)
target_link_libraries(${PROJECT_NAME}_util
  ${catkin_LIBRARIES}
)

# pca9685 library
add_library(${PROJECT_NAME}_pca9685 C++/Navio/PCA9685.cpp)
target_link_libraries(${PROJECT_NAME}_pca9685
  ${catkin_LIBRARIES}
)

# i2cdev library
add_library(${PROJECT_NAME}_i2cdev C++/Navio/I2Cdev.cpp)
target_link_libraries(${PROJECT_NAME}_i2cdev
  ${catkin_LIBRARIES}
)


# AHRS executable
add_executable(${PROJECT_NAME}_ahrs src/AHRS.cpp)
set_target_properties(${PROJECT_NAME}_ahrs PROPERTIES OUTPUT_NAME ahrs PREFIX "")
target_link_libraries(${PROJECT_NAME}_ahrs
  ${catkin_LIBRARIES}
  navio_mpu9250
)

# LED executable
add_executable(${PROJECT_NAME}_led src/navio_led.cpp)
set_target_properties(${PROJECT_NAME}_led PROPERTIES OUTPUT_NAME led PREFIX "")
target_link_libraries(${PROJECT_NAME}_led
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_gpio
  ${PROJECT_NAME}_util
  ${PROJECT_NAME}_pca9685
  ${PROJECT_NAME}_i2cdev
)

## Install ------------------------------------------------------------

install(TARGETS
	${PROJECT_NAME}_ahrs
	${PROJECT_NAME}_led
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
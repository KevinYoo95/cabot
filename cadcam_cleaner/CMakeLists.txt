# Copyright(c) 2020 eaibot limited.
cmake_minimum_required(VERSION 2.8)

project(cadcam_cleaner)


############## YDLIDAR SDK START#####################################
#find ydlidar_sdk package
find_package(ydlidar_sdk REQUIRED)

############## YDLIDAR SDK END#####################################

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  	message_generation
	roscpp
	rospy
	sensor_msgs
        tf
	std_srvs
	)

##################LaserFan ##########################
#add_message_files(DIRECTORY msg
#  FILES
#  LaserFan.msg
#)

#generate_messages(DEPENDENCIES
#  std_msgs
#)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects als    o need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(CATKIN_DEPENDS
	roscpp rospy sensor_msgs message_runtime
	)
#---------------------------------------------------------------------------------------
# Set default build to release
#---------------------------------------------------------------------------------------
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose Release or Debug" FORCE)
endif()

#---------------------------------------------------------------------------------------
# Compiler config
#---------------------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

#Include directories
include_directories(${catkin_INCLUDE_DIRS} ${YDLIDAR_SDK_INCLUDE_DIRS})

#link library directories
link_directories(${YDLIDAR_SDK_LIBRARY_DIRS})



#---------------------------------------------------------------------------------------
# generate excutable and add libraries
#---------------------------------------------------------------------------------------
add_executable(${PROJECT_NAME}_node src/ydlidar_ros_driver.cpp)
add_executable(wallFollowing0812 src/wallFollowing0812.cpp)
add_executable(zigzag_0824 src/zigzag_0824.cpp)
add_executable(wallFollowing0824 src/wallFollowing0824.cpp)
#---------------------------------------------------------------------------------------
# add projects that depend on
#---------------------------------------------------------------------------------------
#add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp)

#---------------------------------------------------------------------------------------
# link libraries
#---------------------------------------------------------------------------------------
target_link_libraries(${PROJECT_NAME}_node
	${catkin_LIBRARIES}
	${YDLIDAR_SDK_LIBRARIES}
	)

	
target_link_libraries(wallFollowing0812
	${catkin_LIBRARIES}
	${YDLIDAR_SDK_LIBRARIES}
	)

target_link_libraries(zigzag_0824
	${catkin_LIBRARIES}
	${YDLIDAR_SDK_LIBRARIES}
	)

target_link_libraries(wallFollowing0824
	${catkin_LIBRARIES}
	${YDLIDAR_SDK_LIBRARIES}
	)
#---------------------------------------------------------------------------------------
# Install
#---------------------------------------------------------------------------------------

install(TARGETS ${PROJECT_NAME}_node wallFollowing0812
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)
install(TARGETS ${PROJECT_NAME}_node zigzag_0824
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)
install(TARGETS ${PROJECT_NAME}_node wallFollowing0824
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)

cmake_minimum_required(VERSION 2.4)
project(pointcloud_manipulation)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  sensor_msgs
  tf2_sensor_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  #INCLUDE_DIRS
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  roscpp
  sensor_msgs
  tf2_sensor_msgs
  geometry_msgs
)

###########
## Build ##
###########


set(TARGET_NAME pointcloud_manipulation)

add_executable(${TARGET_NAME} src/pointcloud_manipulation.cpp)

target_include_directories(${TARGET_NAME}
	PUBLIC ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${TARGET_NAME}
	${catkin_LIBRARIES}
	${roscpp_LIBRARIES} 
	${tf2_LIBRARIES}
	
)

MESSAGE( STATUS "variable is: " ${tf_LIBRARIES} )
MESSAGE( STATUS "variable is: " ${catkin_LIBRARIES} )

	

#############
## Install ##
#############

install(TARGETS ${TARGET_NAME} DESTINATION bin)

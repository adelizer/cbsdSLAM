cmake_minimum_required(VERSION 2.8.3)
project(slam_composites)


include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


find_package(MRPT REQUIRED base;bayes;obs;gui;slam;vision;topography)
find_package(Eigen3 REQUIRED)


set(FLIRT_DIR /home/adel/workspace/external/flirtlib/build/)
find_package(FLIRT REQUIRED)


set(aruco_DIR /home/adel/workspace/external/aruco-2.0.19/build/)
find_package(aruco REQUIRED)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  laser_geometry
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  image_transport
  cv_bridge
)



catkin_package(

)

###########
## Build ##
###########


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  "/usr/include/eigen3/"
  ${OpenCV_INCLUDE_DIRs}
${FLIRT_INCLUDE_DIRS}
)

include_directories( /home/adel/workspace/slam/primitives/mrptEKF/src )
include_directories( /home/adel/workspace/slam/primitives/pf/src )
include_directories( /home/adel/workspace/slam/primitives/depthFeatures/src )
include_directories( /home/adel/workspace/slam/primitives/visionFeatures/src )
include_directories( /home/adel/workspace/slam/primitives/gridmapViz/src )

file(GLOB EKF_SOURCES /home/adel/workspace/slam/primitives/mrptEKF/src/*.cpp)
file(GLOB PF_SOURCES /home/adel/workspace/slam/primitives/pf/src/*.cpp)
file(GLOB VISION_SOURCES /home/adel/workspace/slam/primitives/visionFeatures/src/*.cpp)
file(GLOB DEPTH_SOURCES /home/adel/workspace/slam/primitives/depthFeatures/src/*.cpp)
file(GLOB GRID_SOURCES /home/adel/workspace/slam/primitives/gridmapViz/src/*.cpp)



# particle filter with grid based map and platform odometry
add_executable(${PROJECT_NAME}_pf_grid src/pf_grid.cpp ${PF_SOURCES})

target_link_libraries(${PROJECT_NAME}_pf_grid
   ${catkin_LIBRARIES}
	${MRPT_LIBS}
	${FLIRT_LIBRARIES}
${OpenCV_LIBS} ${aruco_LIBS} 
 )

# particle filter with feature based map and visual odometry
add_executable(${PROJECT_NAME}_pf_vo_aruco src/pf_vo_aruco.cpp ${PF_SOURCES} ${VISION_SOURCES})

target_link_libraries(${PROJECT_NAME}_pf_vo_aruco
   ${catkin_LIBRARIES}
	${MRPT_LIBS}
	${FLIRT_LIBRARIES}
${OpenCV_LIBS} ${aruco_LIBS} 
 )


# particle filter with feature based map and visual odometry
add_executable(${PROJECT_NAME}_ekf_vo src/ekf_vo.cpp ${EKF_SOURCES} ${VISION_SOURCES})

target_link_libraries(${PROJECT_NAME}_ekf_vo
   ${catkin_LIBRARIES}
	${MRPT_LIBS}
	${FLIRT_LIBRARIES}
${OpenCV_LIBS} ${aruco_LIBS} 
 )


# EKF with feature based map and grid visualization
add_executable(${PROJECT_NAME}_ekf_odom_grid src/ekf_odom_grid.cpp ${EKF_SOURCES} ${VISION_SOURCES} ${GRID_SOURCES})

target_link_libraries(${PROJECT_NAME}_ekf_odom_grid
   ${catkin_LIBRARIES}
	${MRPT_LIBS}
	${FLIRT_LIBRARIES}
${OpenCV_LIBS} ${aruco_LIBS} 
 )


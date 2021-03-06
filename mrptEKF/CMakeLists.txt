cmake_minimum_required(VERSION 2.8.3)
project(mrptEKF_slam)


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

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(MRPT REQUIRED base;bayes;obs;gui;slam;vision;topography)
find_package(Eigen3 REQUIRED)


## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories("/usr/include/eigen3/")

include_directories(src include)

file(GLOB SOURCES src/*.cpp)
file(GLOB TESTS tests/*.cpp)

add_executable(test_mrpt_ekf  ${TESTS} ${SOURCES})
target_link_libraries(test_mrpt_ekf  ${MRPT_LIBS} )


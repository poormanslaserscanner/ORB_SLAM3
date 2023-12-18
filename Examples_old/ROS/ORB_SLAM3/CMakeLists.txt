cmake_minimum_required(VERSION 3.16)
MESSAGE("CMAKE_PREFIX_PATH: '${CMAKE_PREFIX_PATH}'")
set(SUPERBUILD_PREFIX_PATH ${CMAKE_PREFIX_PATH})
unset(CMAKE_PREFIX_PATH CACHE)
MESSAGE("CMAKE_PREFIX_PATH: '${CMAKE_PREFIX_PATH}'")

MESSAGE("CMAKE_PROJECT_NAME: " ${CMAKE_PROJECT_NAME})
MESSAGE("PROJECT_NAME: " ${PROJECT_NAME})

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()


get_property(not_top DIRECTORY PROPERTY PARENT_DIRECTORY)
if (NOT not_top)
	set(CMAKE_CXX_STANDARD 14)
	set(CMAKE_CXX_STANDARD_REQUIRED ON)
	set(CMAKE_C_EXTENSIONS OFF)
	set(CMAKE_CXX_EXTENSIONS OFF)
endif ()

if(MSVC)
    set(CMAKE_DEBUG_POSTFIX "_d")
endif()


#IF(NOT CMAKE_BUILD_TYPE)
#  SET(CMAKE_BUILD_TYPE Release)
#ENDIF()


MESSAGE("Cmake Build type: " ${CMAKE_BUILD_TYPE})



MESSAGE("CMAKE_PROJECT_NAME: " ${CMAKE_PROJECT_NAME})
MESSAGE("PROJECT_NAME: " ${PROJECT_NAME})

message("PROJECT_BINARY_DIR: '${PROJECT_BINARY_DIR}'")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
IF(NOT ROS_BUILD_TYPE)
  SET(ROS_BUILD_TYPE Release)
ENDIF()

MESSAGE("ROS Build type: " ${ROS_BUILD_TYPE})

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

set(CMAKE_PREFIX_PATH ${SUPERBUILD_PREFIX_PATH} ${CMAKE_PREFIX_PATH})
MESSAGE("CMAKE_PREFIX_PATH: '${CMAKE_PREFIX_PATH}'")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
#   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++14.")
elseif(COMPILER_SUPPORTS_CXX0X)
#   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake_modules)

#find_package(OpenCV 4.2 REQUIRED)
#if(NOT OpenCV_FOUND)
#   find_package(OpenCV 2.4.3 QUIET)
#   if(NOT OpenCV_FOUND)
#      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
#   endif()
#endif()

#find_package(Eigen3 3.1.0 REQUIRED)
#find_package(Pangolin REQUIRED)

#include_directories(
#${PROJECT_SOURCE_DIR}
#${PROJECT_SOURCE_DIR}/../../../
#${PROJECT_SOURCE_DIR}/../../../Thirdparty/DBoW2
#${PROJECT_SOURCE_DIR}/../../../Thirdparty/Sophus
#${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o
#${PROJECT_SOURCE_DIR}/../../../../../Build/ORB_SLAM3/g2o_Build
#${PROJECT_SOURCE_DIR}/../../../include
#${PROJECT_SOURCE_DIR}/../../../include/CameraModels
#${Pangolin_INCLUDE_DIRS}
#)

#set(LIBS 
#${OpenCV_LIBS} 
#${EIGEN3_LIBS}
#${Pangolin_LIBRARIES}
##${PROJECT_SOURCE_DIR}/../../../Thirdparty/DBoW2/lib/libDBoW2.so
#${PROJECT_SOURCE_DIR}/../../../../../Build/ORB_SLAM3/DBoW2_Build/libDBoW2.so
##${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o.so
#${PROJECT_SOURCE_DIR}/../../../../../Build/ORB_SLAM3/g2o_Build/libg2o.so
##${PROJECT_SOURCE_DIR}/../../../lib/libORB_SLAM3.so
#${PROJECT_SOURCE_DIR}/../../../../../Build/ORB_SLAM3/libORB_SLAM3.so
#-lboost_system
#)
#find_package(Pangolin REQUIRED)

#find_package(ORB_SLAM3 REQUIRED)
get_filename_component(ORB_SLAM3_DIR ${PROJECT_SOURCE_DIR}/../../.. ABSOLUTE)
MESSAGE("ORB_SLAM3_DIR: '${ORB_SLAM3_DIR}'")

add_subdirectory(${ORB_SLAM3_DIR} ORB_SLAM3_build)
# Node for monocular camera
rosbuild_add_executable(Mono
src/ros_mono.cc
)

target_link_libraries(Mono ORB_SLAM3::ORB_SLAM3
#${LIBS}
)

# Node for monocular camera (Augmented Reality Demo)
#rosbuild_add_executable(MonoAR
#src/AR/ros_mono_ar.cc
#src/AR/ViewerAR.h
#src/AR/ViewerAR.cc
#)

#target_link_libraries(MonoAR
#${LIBS}
#)

# Node for stereo camera
rosbuild_add_executable(Stereo
src/ros_stereo.cc
)

target_link_libraries(Stereo ORB_SLAM3::ORB_SLAM3
#${LIBS}
)

# Node for RGB-D camera
rosbuild_add_executable(RGBD
src/ros_rgbd.cc
)

target_link_libraries(RGBD ORB_SLAM3::ORB_SLAM3
#${LIBS}
)

# Node for monocular-inertial camera
rosbuild_add_executable(Mono_Inertial
src/ros_mono_inertial.cc
)

target_link_libraries(Mono_Inertial ORB_SLAM3::ORB_SLAM3
#${LIBS}
)

# Node for stereo-inertial camera
rosbuild_add_executable(Stereo_Inertial
src/ros_stereo_inertial.cc
)

target_link_libraries(Stereo_Inertial ORB_SLAM3::ORB_SLAM3
#${LIBS}
)


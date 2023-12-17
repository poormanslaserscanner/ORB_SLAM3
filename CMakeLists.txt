cmake_minimum_required(VERSION 3.16)
project(ORB_SLAM3 VERSION 1.0)

include(GNUInstallDirs)

if (CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
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


MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})



#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")
#set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)


# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
#if(COMPILER_SUPPORTS_CXX11)
##   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#   add_definitions(-DCOMPILEDWITHC11)
#   message(STATUS "Using flag -std=c++11.")
#elseif(COMPILER_SUPPORTS_CXX0X)
##   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
#   add_definitions(-DCOMPILEDWITHC0X)
#   message(STATUS "Using flag -std=c++0x.")
#else()
#   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
#endif()


find_package(OpenCV 4.2)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 4.2 not found.")
   endif()

MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(realsense2)

#include_directories(
#${PROJECT_SOURCE_DIR}
#${PROJECT_SOURCE_DIR}/include
#${PROJECT_SOURCE_DIR}/include/CameraModels
#${PROJECT_SOURCE_DIR}/Thirdparty/Sophus
#${EIGEN3_INCLUDE_DIR}
#${Pangolin_INCLUDE_DIRS}
#)

#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/DBoW2 DBoW2_Build)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/g2o g2o_Build)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/Sophus Sophus_Build)



add_library(${PROJECT_NAME} SHARED
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Atlas.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Viewer.cc
src/ImuTypes.cc
src/G2oTypes.cc
src/CameraModels/Pinhole.cpp
src/CameraModels/KannalaBrandt8.cpp
src/OptimizableTypes.cpp
src/MLPnPsolver.cpp
src/GeometricTools.cc
src/TwoViewReconstruction.cc
src/Config.cc
src/Settings.cc
include/System.h
include/Tracking.h
include/LocalMapping.h
include/LoopClosing.h
include/ORBextractor.h
include/ORBmatcher.h
include/FrameDrawer.h
include/Converter.h
include/MapPoint.h
include/KeyFrame.h
include/Atlas.h
include/Map.h
include/MapDrawer.h
include/Optimizer.h
include/Frame.h
include/KeyFrameDatabase.h
include/Sim3Solver.h
include/Viewer.h
include/ImuTypes.h
include/G2oTypes.h
include/CameraModels/GeometricCamera.h
include/CameraModels/Pinhole.h
include/CameraModels/KannalaBrandt8.h
include/OptimizableTypes.h
include/MLPnPsolver.h
include/GeometricTools.h
include/TwoViewReconstruction.h
include/SerializationUtils.h
include/Config.h
include/Settings.h)



if(COMPILER_SUPPORTS_CXX11)
#   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   target_compile_definitions(${PROJECT_NAME} PUBLIC COMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
#   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   target_compile_definitions(${PROJECT_NAME} PUBLIC COMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

add_library(ORB_SLAM3::ORB_SLAM3 ALIAS ORB_SLAM3)
target_include_directories(ORB_SLAM3
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_include_directories(ORB_SLAM3
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/CameraModels>
#        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/CameraModels>
)

find_package(Pangolin REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC ${Pangolin_LIBRARIES})

#find_package(DBoW2 REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC DBoW2::DBoW2)

#find_package(g2o REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC g2o::g2o)

#find_package(Sophus REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC Sophus::Sophus)

find_package(Boost REQUIRED COMPONENTS serialization)
target_link_libraries(${PROJECT_NAME} PUBLIC ${Boost_LIBRARIES})

MESSAGE("Boost_LIBRARIES: " ${Boost_LIBRARIES})

# If RealSense SDK is found the library is added and its examples compiled
if(realsense2_FOUND)
    message(FATAL_ERROR "realsense2 found.")
    include_directories(${PROJECT_NAME}
    ${realsense_INCLUDE_DIR}
    )
    target_link_libraries(${PROJECT_NAME}
    ${realsense2_LIBRARY}
    )
endif()


# Build examples

# RGB-D examples
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)

add_executable(rgbd_tum
        Examples/RGB-D/rgbd_tum.cc)
target_link_libraries(rgbd_tum ${PROJECT_NAME})
set(all_the_executales rgbd_tum)
if(realsense2_FOUND)
    add_executable(rgbd_realsense_D435i
            Examples/RGB-D/rgbd_realsense_D435i.cc)
    target_link_libraries(rgbd_realsense_D435i ${PROJECT_NAME})
    set(all_the_executales ${all_the_executales} rgbd_realsense_D435i)
endif()


# RGB-D inertial examples
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D-Inertial)

if(realsense2_FOUND)
    add_executable(rgbd_inertial_realsense_D435i
            Examples/RGB-D-Inertial/rgbd_inertial_realsense_D435i.cc)
    target_link_libraries(rgbd_inertial_realsense_D435i ${PROJECT_NAME})
    set(all_the_executales ${all_the_executales} rgbd_inertial_realsense_D435i)
endif()

#Stereo examples
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

add_executable(stereo_kitti
        Examples/Stereo/stereo_kitti.cc)
target_link_libraries(stereo_kitti ${PROJECT_NAME})
set(all_the_executales ${all_the_executales} stereo_kitti)


add_executable(stereo_euroc
        Examples/Stereo/stereo_euroc.cc)
target_link_libraries(stereo_euroc ${PROJECT_NAME})
set(all_the_executales ${all_the_executales} stereo_euroc)


add_executable(stereo_tum_vi
        Examples/Stereo/stereo_tum_vi.cc)
target_link_libraries(stereo_tum_vi ${PROJECT_NAME})
set(all_the_executales ${all_the_executales} stereo_tum_vi)


if(realsense2_FOUND)
    add_executable(stereo_realsense_t265
            Examples/Stereo/stereo_realsense_t265.cc)
    target_link_libraries(stereo_realsense_t265 ${PROJECT_NAME})
    set(all_the_executales ${all_the_executales} stereo_realsense_t265)

    add_executable(stereo_realsense_D435i
            Examples/Stereo/stereo_realsense_D435i.cc)
    target_link_libraries(stereo_realsense_D435i ${PROJECT_NAME})
    set(all_the_executales ${all_the_executales} stereo_realsense_D435i)
endif()

#Monocular examples
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

add_executable(mono_tum
        Examples/Monocular/mono_tum.cc)
target_link_libraries(mono_tum ${PROJECT_NAME})
set(all_the_executales ${all_the_executales} mono_tum)

add_executable(mono_kitti
        Examples/Monocular/mono_kitti.cc)
target_link_libraries(mono_kitti ${PROJECT_NAME})
set(all_the_executales ${all_the_executales} mono_kitti)

add_executable(mono_euroc
        Examples/Monocular/mono_euroc.cc)
target_link_libraries(mono_euroc ${PROJECT_NAME})
set(all_the_executales ${all_the_executales} mono_euroc)

add_executable(mono_tum_vi
        Examples/Monocular/mono_tum_vi.cc)
target_link_libraries(mono_tum_vi ${PROJECT_NAME})
set(all_the_executales ${all_the_executales} mono_tum_vi)

if(realsense2_FOUND)
    add_executable(mono_realsense_t265
            Examples/Monocular/mono_realsense_t265.cc)
    target_link_libraries(mono_realsense_t265 ${PROJECT_NAME})
    set(all_the_executales ${all_the_executales} mono_realsense_t265)

    add_executable(mono_realsense_D435i
            Examples/Monocular/mono_realsense_D435i.cc)
    target_link_libraries(mono_realsense_D435i ${PROJECT_NAME})
    set(all_the_executales ${all_the_executales} mono_realsense_D435i)
endif()

#Monocular inertial examples
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular-Inertial)

add_executable(mono_inertial_euroc
        Examples/Monocular-Inertial/mono_inertial_euroc.cc)
target_link_libraries(mono_inertial_euroc ${PROJECT_NAME})
set(all_the_executales ${all_the_executales} mono_inertial_euroc)

add_executable(mono_inertial_tum_vi
        Examples/Monocular-Inertial/mono_inertial_tum_vi.cc)
target_link_libraries(mono_inertial_tum_vi ${PROJECT_NAME})
set(all_the_executales ${all_the_executales} mono_inertial_tum_vi)

if(realsense2_FOUND)
    add_executable(mono_inertial_realsense_t265
            Examples/Monocular-Inertial/mono_inertial_realsense_t265.cc)
    target_link_libraries(mono_inertial_realsense_t265 ${PROJECT_NAME})
    set(all_the_executales ${all_the_executales} mono_inertial_realsense_t265)

    add_executable(mono_inertial_realsense_D435i
            Examples/Monocular-Inertial/mono_inertial_realsense_D435i.cc)
    target_link_libraries(mono_inertial_realsense_D435i ${PROJECT_NAME})
    set(all_the_executales ${all_the_executales} mono_inertial_realsense_D435i)
endif()

#Stereo Inertial examples
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo-Inertial)

add_executable(stereo_inertial_euroc
        Examples/Stereo-Inertial/stereo_inertial_euroc.cc)
target_link_libraries(stereo_inertial_euroc ${PROJECT_NAME})
set(all_the_executales ${all_the_executales} stereo_inertial_euroc)

add_executable(stereo_inertial_tum_vi
        Examples/Stereo-Inertial/stereo_inertial_tum_vi.cc)
target_link_libraries(stereo_inertial_tum_vi ${PROJECT_NAME})
set(all_the_executales ${all_the_executales} stereo_inertial_tum_vi)

if(realsense2_FOUND)
    add_executable(stereo_inertial_realsense_t265
            Examples/Stereo-Inertial/stereo_inertial_realsense_t265.cc)
    target_link_libraries(stereo_inertial_realsense_t265 ${PROJECT_NAME})
    set(all_the_executales ${all_the_executales} stereo_inertial_realsense_t265)

    add_executable(stereo_inertial_realsense_D435i
            Examples/Stereo-Inertial/stereo_inertial_realsense_D435i.cc)
    target_link_libraries(stereo_inertial_realsense_D435i ${PROJECT_NAME})
    set(all_the_executales ${all_the_executales} stereo_inertial_realsense_D435i)
endif()

#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Calibration)
if(realsense2_FOUND)
    add_executable(recorder_realsense_D435i
            Examples/Calibration/recorder_realsense_D435i.cc)
    target_link_libraries(recorder_realsense_D435i ${PROJECT_NAME})
    set(all_the_executales ${all_the_executales} recorder_realsense_D435i)

    add_executable(recorder_realsense_T265
            Examples/Calibration/recorder_realsense_T265.cc)
    target_link_libraries(recorder_realsense_T265 ${PROJECT_NAME})
    set(all_the_executales ${all_the_executales} recorder_realsense_T265)
endif()

#Old examples

# RGB-D examples
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples_old/RGB-D)

add_executable(rgbd_tum_old
        Examples_old/RGB-D/rgbd_tum.cc)
target_link_libraries(rgbd_tum_old ${PROJECT_NAME})
set(all_the_executales ${all_the_executales} rgbd_tum_old)

if(realsense2_FOUND)
    add_executable(rgbd_realsense_D435i_old
            Examples_old/RGB-D/rgbd_realsense_D435i.cc)
    target_link_libraries(rgbd_realsense_D435i_old ${PROJECT_NAME})
    set(all_the_executales ${all_the_executales} rgbd_realsense_D435i_old)
endif()


# RGB-D inertial examples
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples_old/RGB-D-Inertial)

if(realsense2_FOUND)
    add_executable(rgbd_inertial_realsense_D435i_old
            Examples_old/RGB-D-Inertial/rgbd_inertial_realsense_D435i.cc)
    target_link_libraries(rgbd_inertial_realsense_D435i_old ${PROJECT_NAME})
    set(all_the_executales ${all_the_executales} rgbd_inertial_realsense_D435i_old)
endif()

#Stereo examples
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples_old/Stereo)

add_executable(stereo_kitti_old
        Examples_old/Stereo/stereo_kitti.cc)
target_link_libraries(stereo_kitti_old ${PROJECT_NAME})
set(all_the_executales ${all_the_executales} stereo_kitti_old)

add_executable(stereo_euroc_old
        Examples_old/Stereo/stereo_euroc.cc)
target_link_libraries(stereo_euroc_old ${PROJECT_NAME})
set(all_the_executales ${all_the_executales} stereo_euroc_old)

add_executable(stereo_tum_vi_old
        Examples_old/Stereo/stereo_tum_vi.cc)
target_link_libraries(stereo_tum_vi_old ${PROJECT_NAME})
set(all_the_executales ${all_the_executales} stereo_tum_vi_old)

if(realsense2_FOUND)
    add_executable(stereo_realsense_t265_old
            Examples_old/Stereo/stereo_realsense_t265.cc)
    target_link_libraries(stereo_realsense_t265_old ${PROJECT_NAME})
set(all_the_executales ${all_the_executales} stereo_realsense_t265_old)

    add_executable(stereo_realsense_D435i_old
            Examples_old/Stereo/stereo_realsense_D435i.cc)
    target_link_libraries(stereo_realsense_D435i_old ${PROJECT_NAME})
set(all_the_executales ${all_the_executales} stereo_realsense_D435i_old)
endif()

#Monocular examples
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples_old/Monocular)

add_executable(mono_tum_old
        Examples_old/Monocular/mono_tum.cc)
target_link_libraries(mono_tum_old ${PROJECT_NAME})
set(all_the_executales ${all_the_executales} mono_tum_old)

add_executable(mono_kitti_old
        Examples_old/Monocular/mono_kitti.cc)
target_link_libraries(mono_kitti_old ${PROJECT_NAME})
set(all_the_executales ${all_the_executales} mono_kitti_old)

add_executable(mono_euroc_old
        Examples_old/Monocular/mono_euroc.cc)
target_link_libraries(mono_euroc_old ${PROJECT_NAME})
set(all_the_executales ${all_the_executales} mono_euroc_old)

add_executable(mono_tum_vi_old
        Examples_old/Monocular/mono_tum_vi.cc)
target_link_libraries(mono_tum_vi_old ${PROJECT_NAME})
set(all_the_executales ${all_the_executales} mono_tum_vi_old)

if(realsense2_FOUND)
    add_executable(mono_realsense_t265_old
            Examples_old/Monocular/mono_realsense_t265.cc)
    target_link_libraries(mono_realsense_t265_old ${PROJECT_NAME})
    set(all_the_executales ${all_the_executales} mono_realsense_t265_old)

    add_executable(mono_realsense_D435i_old
            Examples_old/Monocular/mono_realsense_D435i.cc)
    target_link_libraries(mono_realsense_D435i_old ${PROJECT_NAME})
    set(all_the_executales ${all_the_executales} mono_realsense_D435i_old)
endif()

#Monocular inertial examples
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples_old/Monocular-Inertial)

add_executable(mono_inertial_euroc_old
        Examples_old/Monocular-Inertial/mono_inertial_euroc.cc)
target_link_libraries(mono_inertial_euroc_old ${PROJECT_NAME})
set(all_the_executales ${all_the_executales} mono_inertial_euroc_old)

add_executable(mono_inertial_tum_vi_old
        Examples_old/Monocular-Inertial/mono_inertial_tum_vi.cc)
target_link_libraries(mono_inertial_tum_vi_old ${PROJECT_NAME})
set(all_the_executales ${all_the_executales} mono_inertial_tum_vi_old)

if(realsense2_FOUND)
    add_executable(mono_inertial_realsense_t265_old
            Examples_old/Monocular-Inertial/mono_inertial_realsense_t265.cc)
    target_link_libraries(mono_inertial_realsense_t265_old ${PROJECT_NAME})
    set(all_the_executales ${all_the_executales} mono_inertial_realsense_t265_old)

    add_executable(mono_inertial_realsense_D435i_old
            Examples_old/Monocular-Inertial/mono_inertial_realsense_D435i.cc)
    target_link_libraries(mono_inertial_realsense_D435i_old ${PROJECT_NAME})
    set(all_the_executales ${all_the_executales} mono_inertial_realsense_D435i_old)
endif()

#Stereo Inertial examples
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples_old/Stereo-Inertial)

add_executable(stereo_inertial_euroc_old
        Examples_old/Stereo-Inertial/stereo_inertial_euroc.cc)
target_link_libraries(stereo_inertial_euroc_old ${PROJECT_NAME})
set(all_the_executales ${all_the_executales} stereo_inertial_euroc_old)

add_executable(stereo_inertial_tum_vi_old
        Examples_old/Stereo-Inertial/stereo_inertial_tum_vi.cc)
target_link_libraries(stereo_inertial_tum_vi_old ${PROJECT_NAME})
set(all_the_executales ${all_the_executales} stereo_inertial_tum_vi_old)

if(realsense2_FOUND)
    add_executable(stereo_inertial_realsense_t265_old
            Examples_old/Stereo-Inertial/stereo_inertial_realsense_t265.cc)
    target_link_libraries(stereo_inertial_realsense_t265_old ${PROJECT_NAME})
    set(all_the_executales ${all_the_executales} stereo_inertial_realsense_t265_old)

    add_executable(stereo_inertial_realsense_D435i_old
            Examples_old/Stereo-Inertial/stereo_inertial_realsense_D435i.cc)
    target_link_libraries(stereo_inertial_realsense_D435i_old ${PROJECT_NAME})
    set(all_the_executales ${all_the_executales} stereo_inertial_realsense_D435i_old)
endif()

MESSAGE("CMAKE_PROJECT_NAME: " ${CMAKE_PROJECT_NAME})
MESSAGE("PROJECT_NAME: " ${PROJECT_NAME})
get_property(not_top DIRECTORY PROPERTY PARENT_DIRECTORY)
if(NOT not_top)
#if (CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
		if (WIN32)
			set (shext bat)
			file(COPY ${CMAKE_BINARY_DIR}/../../../run/start.lnk DESTINATION ${CMAKE_BINARY_DIR}/../run)
		elseif(UNIX)
			set (shext sh)
			file(COPY ${CMAKE_BINARY_DIR}/../../../run/start.sh DESTINATION ${CMAKE_BINARY_DIR}/../run)
		endif()
		file(COPY ${CMAKE_BINARY_DIR}/../../../run/setvars.${shext} DESTINATION ${CMAKE_BINARY_DIR}/../run)
		if (WIN32)
			file(APPEND ${CMAKE_BINARY_DIR}/../run/setvars.${shext} "call setrunenv.${shext}")
		else()
			file(APPEND ${CMAKE_BINARY_DIR}/../run/setvars.${shext} ". setrunenv.${shext}\n")
		endif()
#		if (OpenCV_INSTALL_PATH)
#			if (MSVC)
#				set(OpencvCmakeConfigFile "OpenCVConfig.cmake")
#			else()
#				set(OpencvCmakeConfigFile "lib64/cmake/opencv4/OpenCVConfig.cmake")
#			endif()
#			include(${OpenCV_INSTALL_PATH}/${OpencvCmakeConfigFile})
#			if (OpenCV_LIB_PATH)
#				get_filename_component(lib_tmp "${OpenCV_LIB_PATH}/../bin" ABSOLUTE)
#				set(DVTK_RUN_PATH ${lib_tmp} ${DVTK_RUN_PATH})
#			endif()
#		endif()
		set (prefix_tmp ${CMAKE_PREFIX_PATH})
		list(TRANSFORM prefix_tmp APPEND /bin)
		set(DVTK_RUN_PATH ${prefix_tmp} ${DVTK_RUN_PATH})
		set (prefix_tmp ${CMAKE_PREFIX_PATH})
		list(TRANSFORM prefix_tmp APPEND /lib)
		set(DVTK_RUN_PATH ${prefix_tmp} ${DVTK_RUN_PATH})
		if (MSVC)
			set(DVTK_RUN_PATH ${CMAKE_CURRENT_BINARY_DIR}/Debug ${DVTK_RUN_PATH})
			set(DVTK_RUN_PATH ${CMAKE_CURRENT_BINARY_DIR}/Release ${DVTK_RUN_PATH})
		else ()
			set(DVTK_RUN_PATH ${CMAKE_CURRENT_BINARY_DIR} ${DVTK_RUN_PATH})
		endif()
		file(TO_NATIVE_PATH "${DVTK_RUN_PATH}" DVTK_RUN_PATH)
		if (WIN32)
			configure_file(${CMAKE_CURRENT_LIST_DIR}/setrunenv.${shext}.in ${CMAKE_BINARY_DIR}/../run/setrunenv.${shext})
		elseif (UNIX)
			string(REPLACE ";" ":" DVTK_RUN_PATH "${DVTK_RUN_PATH}")
			message("DVTK_RUN_PATH: '${DVTK_RUN_PATH}'")
			configure_file(${CMAKE_CURRENT_LIST_DIR}/setrunenv.${shext}.in ${CMAKE_BINARY_DIR}/../run/tmp/setrunenv.${shext} @ONLY)
			file (COPY ${CMAKE_BINARY_DIR}/../run/tmp/setrunenv.${shext} DESTINATION ${CMAKE_BINARY_DIR}/../run
				FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
		endif()
		if (CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
			message("startIDE.${shext} created from project ${PROJECT_NAME}")
			file(COPY ${CMAKE_CURRENT_LIST_DIR}/startIDE.${shext} DESTINATION ${CMAKE_BINARY_DIR}/../run)
		endif()
		file(COPY ${CMAKE_CURRENT_LIST_DIR}/help.${shext} DESTINATION ${CMAKE_BINARY_DIR}/../run)
endif()


set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/ORB_SLAM3)

install(
    TARGETS
      ORB_SLAM3 ${all_the_executales}
    EXPORT ORB_SLAM3Targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ORB_SLAM3 FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")
if (MSVC)
    install(FILES $<TARGET_PDB_FILE:${PROJECT_NAME}> DESTINATION bin OPTIONAL)
endif()

install(EXPORT ORB_SLAM3Targets
    FILE ORB_SLAM3Targets.cmake
    NAMESPACE ORB_SLAM3::
    DESTINATION ${INSTALL_CONFIGDIR}
)


#####################
# ConfigVersion file
##
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/ORB_SLAM3ConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(
    ${CMAKE_CURRENT_LIST_DIR}/CMake/ORB_SLAM3Config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/ORB_SLAM3Config.cmake
    INSTALL_DESTINATION ${INSTALL_CONFIGDIR}
)

## Install all the helper files
install(
    FILES
      ${CMAKE_CURRENT_BINARY_DIR}/ORB_SLAM3Config.cmake
      ${CMAKE_CURRENT_BINARY_DIR}/ORB_SLAM3ConfigVersion.cmake
    DESTINATION ${INSTALL_CONFIGDIR}
)


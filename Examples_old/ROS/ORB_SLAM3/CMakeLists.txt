cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

IF(NOT ROS_BUILD_TYPE)
  SET(ROS_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${ROS_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -mtune=generic ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -mtune=generic")

# Set C++14 standardset(CMAKE_CXX_STANDARD 14)set(CMAKE_CXX_STANDARD_REQUIRED ON)set(CMAKE_CXX_EXTENSIONS OFF)



#find_package(OpenCV 4.0 REQUIRED)
find_package(OpenCV 4.2 EXACT REQUIRED COMPONENTS core imgproc imgcodecs highgui features2d calib3d)
if(NOT OpenCV_FOUND)
#   find_package(OpenCV 2.4.3 QUIET)
#   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 4.4 not found.")
#   endif()
endif()

#find_package(PkgConfig QUIET)
#if(PkgConfig_FOUND)
#  pkg_check_modules(OPENCV4 QUIET opencv4)
#  if(OPENCV4_FOUND)
#    message(STATUS "Using pkg-config opencv4 override: ${OPENCV4_VERSION}")
#    set(OpenCV_INCLUDE_DIRS ${OPENCV4_INCLUDE_DIRS})
#    set(OpenCV_LIBS ${OPENCV4_LIBRARIES})
#  endif()
#endif()

find_package(Eigen3 3.3.7 REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/../../../
${PROJECT_SOURCE_DIR}/../../../Thirdparty/Sophus
${PROJECT_SOURCE_DIR}/../../../include
${PROJECT_SOURCE_DIR}/../../../include/CameraModels
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
)

get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "@@@@@@dir='${dir}'")
endforeach()

#LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake_modules)


set(LIBS 
${OpenCV_LIBS} 
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/../../../Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o.so
${PROJECT_SOURCE_DIR}/../../../lib/libORB_SLAM3.so
-lboost_system
)

# Node for monocular camera
#rosbuild_add_executable(Mono
#src/ros_mono.cc
#)

#target_link_libraries(Mono
#${LIBS}
#)

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

target_link_libraries(Stereo
${LIBS}
)

# Node for RGB-D camera
rosbuild_add_executable(RGBD
src/ros_rgbd.cc
)

target_link_libraries(RGBD
${LIBS}
)

# Node for monocular-inertial camera
rosbuild_add_executable(Mono_Inertial
src/ros_mono_inertial.cc
)

target_link_libraries(Mono_Inertial
${LIBS}
)


# Node for stereo-inertial camera
rosbuild_add_executable(Stereo_Inertial
src/ros_stereo_inertial.cc
elastic/harmonic.cc
elastic/task.cc
)

target_include_directories(Stereo_Inertial PRIVATE elastic)

target_link_libraries(Stereo_Inertial
${LIBS}
)

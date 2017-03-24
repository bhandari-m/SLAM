cmake_minimum_required(VERSION 2.6)
project(landmarkISAM)

set(GTSAM_DIR "/home/Downloads/gtsam-3.2.1/build")
find_package(GTSAM)
include_directories(${GTSAM_INCLUDE_DIR})
include_directories(${GTSAM_DIR})
link_directories(${GTSAM_LIBRARY_DIRS})

add_executable(landmarkISAM landmarkISAM.cpp)
target_link_libraries(landmarkISAM gtsam)

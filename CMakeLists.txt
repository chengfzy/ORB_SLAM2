cmake_minimum_required(VERSION 3.2)
project(ORB_SLAM2)
set(CMAKE_CXX_STANDARD 14)

# build type
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()
MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")

# cmake modules
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# library
find_package(Eigen3 REQUIRED)   # Eigen
find_package(OpenCV REQUIRED)   # OpenCV
find_package(Pangolin REQUIRED) # Pangolin

# include folder
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}
    ${EIGEN3_INCLUDE_DIR}       # Eigen
    ${OpenCV_INCLUDE_DIRS}      # OpenCV
    ${Pangolin_INCLUDE_DIRS}    # Pangolin
)


# G2O
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

# extra librarys
set(EXTRA_LIBS ${EXTRA_LIBS} ${OpenCV_LIBS} ${Pangolin_LIBRARIES}
    #${G2O_LIBS}
#    g2o_core g2o_types_slam3d g2o_solver_csparse g2o_solver_dense g2o_solver_cholmod g2o_stuff g2o_csparse_extension
#    g2o_types_sba g2o_types_sim3 g2o_solver_structure_only
    ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
    )


# project setting: output folder
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)


# project source files
aux_source_directory(./include DIR_INCLUDES)
aux_source_directory(./src DIR_SRCS)

# build library
add_library(${PROJECT_NAME} SHARED
    ${DIR_INCLUDES} ${DIR_SRCS}
)
target_link_libraries(${PROJECT_NAME}
    ${EXTRA_LIBS}
)

# Build examples: RGB-D
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)
#add_executable(rgbd_tum Examples/RGB-D/rgbd_tum.cc)
#target_link_libraries(rgbd_tum ${PROJECT_NAME})

# Build examples: Stero
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)
#add_executable(stereo_kitti Examples/Stereo/stereo_kitti.cc)
#target_link_libraries(stereo_kitti ${PROJECT_NAME})

add_executable(stereo_euroc Examples/Stereo/stereo_euroc.cc)
target_link_libraries(stereo_euroc ${PROJECT_NAME})

# Build examples: Monocular
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)
#add_executable(mono_tum Examples/Monocular/mono_tum.cc)
#target_link_libraries(mono_tum ${PROJECT_NAME})

#add_executable(mono_kitti Examples/Monocular/mono_kitti.cc)
#target_link_libraries(mono_kitti ${PROJECT_NAME})

#add_executable(mono_euroc Examples/Monocular/mono_euroc.cc)
#target_link_libraries(mono_euroc ${PROJECT_NAME})


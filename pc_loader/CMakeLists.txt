# CMakeLists.txt for project B

cmake_minimum_required(VERSION 3.10)
project(pc_loader LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_FLAGS -pthread)


#FIND_PACKAGE(CUDA REQUIRED)
SET(CUDA_INCLUDE_DIRS "/usr/local/cuda/include" ${CUDA_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${CUDA_INCLUDE_DIRS})

if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "No build type selected, default to Release")
  set(CMAKE_BUILD_TYPE "Release")
endif()

set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++17" )

#set( CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -DMY_DEF=1")
#set( CUDA_CXX_FLAGS "${CUDA_CXX_FLAGS} -DMY_DEF=1")
#set( CUDA_C_FLAGS "${CUDA_C_FLAGS} -DMY_DEF=1")

# Include project A as a subdirectory
add_subdirectory(lib/decoders/velodyne_decoder)

#add_library(${PROJECT_NAME} STATIC src/test.cpp)
add_library(${PROJECT_NAME} STATIC
    #lib/decoders/velodyne_decoder/src
    #lib/decoders/velodyne_decoder/src/packet_decoder.cpp
    #lib/decoders/velodyne_decoder/src/scan_decoder.cpp
    #lib/decoders/velodyne_decoder/src/stream_decoder.cpp
    #lib/decoders/velodyne_decoder/src/input.cpp
    src/velodyne_reader.cpp
    src/point_cloud_gpu.cpp
    #src/ops.cu
)

add_library(custom_cuda_libs STATIC src/ops.cu)

target_include_directories(custom_cuda_libs PUBLIC 
  include
  lib/decoders/velodyne_decoder/include
)

target_include_directories(pc_loader PUBLIC 
  include
  lib/decoders/velodyne_decoder/include 
)

set_target_properties( ${PROJECT_NAME}
                       PROPERTIES CUDA_SEPARABLE_COMPILATION ON)


add_executable(pc_read_data src/pc_read_data.cpp)
#add_executable(ops src/ops.cu)
#CUDA_ADD_EXECUTABLE(${PROJECTNAME} ${PROJECT_SRCS})

#target_include_directories(pc_read_data PUBLIC lib/decoders/velodyne_decoder)

# Now you can link project A to project B
target_link_libraries(pc_read_data 
 ${PROJECT_NAME} 
    velodyne_decoder
    boost_system
    boost_thread
    pcap
    ${CUDA_LIBRARIES}
    custom_cuda_libs
)

#target_link_libraries(read_data
#${PROJECT_NAME}
#  boost_system
#  boost_thread
#  pcap
#)



cmake_minimum_required(VERSION 2.8)
add_compile_options(-std=c++0x)
set(ProjectName Autonomous_RC_Car)
project(${ProjectName})
find_package(CUDA QUIET REQUIRED)
# Pass options to NVCC
set(
    CUDA_NVCC_FLAGS
    ${CUDA_NVCC_FLAGS};
    -O3 -gencode arch=compute_20,code=sm_21
    )
# Specify include directories
include_directories(
    kernels
    utility
    )

find_package( PCL 1.2 REQUIRED )

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package( OpenCV REQUIRED )

find_package(Boost COMPONENTS filesystem REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

file(GLOB Project_SRC main.cpp StereoPair.h StereoPair.cpp)
cuda_add_executable(${ProjectName} ${Project_SRC})
target_link_libraries(${ProjectName} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})

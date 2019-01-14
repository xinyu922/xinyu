#cmake_minimum_required(VERSION 3.9)
cmake_minimum_required(VERSION 3.5)
project(Vicon_BiasEstimate)

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_BUILD_TYPE "Debug")

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread -fopenmp")

find_package(OpenCV REQUIRED)

include_directories(
        /usr/include/eigen3
        /usr/local/include/
        ${OpenCV_INCLUDE_DIRS}
        ${CMAKE_SOURCE_DIR}/include
)

find_package(Ceres REQUIRED)

include_directories(${CERES_INCLUDE_DIRS})

find_package(Sophus REQUIRED)

include_directories(${Sohpus_INCLUDE_DIRS})

find_package(Pangolin REQUIRED)

include_directories(${Pangolin_INCLUDE_DIRS})


add_executable(Vicon_BiasEstimate main.cpp
        utility/AHRS.cpp
        estimator.cpp
        factor/pose_local_parameterization.cpp
        factor/quat_local_parameterization.cpp
        parameters.cpp
        )

target_link_libraries(Vicon_BiasEstimate
        ${OpenCV_LIBS}
        ${CERES_LIBRARIES}
        ${Sophus_LIBRARIES}
        ${Pangolin_LIBRARIES}
        )

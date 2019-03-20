cmake_minimum_required(VERSION 2.8)
project(zed_data_capture)

set(CMAKE_CXX_STANDARD 11)

find_package(ZED 2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)

include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})
include_directories(${OPENCV_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

link_directories(${ZED_LIBRARY_DIR})
link_directories(${OPENCV_LIBRARY_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})

message(${ZED_DIR})
message(${ZED_LIBRARIES})

add_executable(zed_data_capture src/main_0.cpp)
target_link_libraries(zed_data_capture ${ZED_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${CUDA_CUDA_LIBRARY}
    ${CUDA_CUDART_LIBRARY}
    ${CUDA_NPP_LIBRARIES_ZED}
)


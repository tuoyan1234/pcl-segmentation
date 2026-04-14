cmake_minimum_required(VERSION 3.10)
project(segmentation)

set(CMAKE_CXX_STANDARD 17)
find_package(PCL 1.10 REQUIRED COMPONENTS
    common
    io
    segmentation
    visualization
    filters
    sample_consensus
)
find_package(VTK REQUIRED)
include_directories(${VTK_INCLUDE_DIRS})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(segmentation main.cpp ${PROJECT_SOURCE_DIR}/src/segmentation.cpp)
target_include_directories(segmentation  PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(segmentation ${PCL_LIBRARIES} pcl_filters yaml-cpp ${VTK_LIBRARIES})
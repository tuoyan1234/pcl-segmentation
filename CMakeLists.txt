cmake_minimum_required(VERSION 3.10)
project(segmentation)

set(CMAKE_CXX_STANDARD 17)
find_package(PCL 1.10 REQUIRED
    COMPONENTS
    common
    io
    segmentation
    visualization
)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# include_directories(${PCL_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/include)

add_executable(segmentation main.cpp ${CMAKE_SOURCE_DIR}/src/segmentation.cpp)

target_include_directories(segmentation  PRIVATE ${PROJECT_SOURCE_DIR}/include)

target_link_libraries(segmentation ${PCL_LIBS} pcl_filters yaml-cpp)
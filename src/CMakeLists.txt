cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(PCL-practice)
find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(samples)

add_executable(read_pcd read_pcd.cpp)
target_link_libraries(read_pcd ${PCL_LIBRARIES})
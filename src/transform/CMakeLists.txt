cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(PCL-practice)
find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(samples)

add_executable(transform transform.cpp)
target_link_libraries(transform ${PCL_LIBRARIES})
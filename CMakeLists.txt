cmake_minimum_required(VERSION 3.5 FATAL_ERROR)


project(counting)


find_package(PCL 1.8 REQUIRED)


add_executable (statistical_removal statistical_removal.cpp)
add_executable (counting counting.cpp)

target_link_libraries (statistical_removal ${PCL_LIBRARIES})
target_link_libraries (counting ${PCL_LIBRARIES})

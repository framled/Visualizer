cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(Visualizer)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (Visualizer Visualizer.cpp 
							Utils/Utilities.cpp 
							Utils/Viewer.cpp 
							Utils/SimpleOpenNIViewer.cpp)
target_link_libraries (Visualizer ${PCL_LIBRARIES})

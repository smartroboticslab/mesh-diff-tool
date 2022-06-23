cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
project(mesh_difference_tool)

set (CMAKE_CXX_STANDARD 17)

find_package(OpenMP)
if(OPENMP_FOUND)
	add_compile_options(${OpenMP_CXX_FLAGS})
	link_libraries(${OpenMP_CXX_LIBRARIES})
	message(STATUS "Compiling with OpenMP support")
else()
	message(WARNING "OpenMP not found. Performance may be terrible.")
endif()

find_package(PCL 1.10 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(tinycolormap/include)
include_directories(include)

add_executable (compareMultiple src/options.cpp src/mesh_properties.cpp src/compare_multiple.cpp src/MeshDifference.cpp)
target_link_libraries (compareMultiple ${PCL_LIBRARIES})

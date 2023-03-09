cmake_minimum_required(VERSION 3.0.2 FATAL_ERROR)
project(mesh-diff-tool
    VERSION 1.0.0
    DESCRIPTION "Compute the accuracy and completeness of 3D mesh reconstructions"
    HOMEPAGE_URL https://github.com/smartroboticslab/mesh-diff-tool.git
    LANGUAGES CXX
)

find_package(OpenMP)
find_package(PCL REQUIRED COMPONENTS common io)

add_library(mesh-diff STATIC src/MeshDifference.cpp src/mesh_properties.cpp src/options.cpp)
target_compile_features(mesh-diff PUBLIC cxx_std_17)
target_compile_definitions(mesh-diff PUBLIC ${PCL_DEFINITIONS})
target_include_directories(mesh-diff
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${PCL_INCLUDE_DIRS}
        tinycolormap/include
)
target_link_directories(mesh-diff PUBLIC ${PCL_LIBRARY_DIRS})
target_link_libraries(mesh-diff PUBLIC stdc++fs ${PCL_LIBRARIES})
if(OPENMP_FOUND)
    target_compile_options(mesh-diff PUBLIC ${OpenMP_CXX_FLAGS})
    target_link_libraries(mesh-diff PUBLIC ${OpenMP_CXX_LIBRARIES})
    message(STATUS "Compiling with OpenMP support")
else()
	message(WARNING "OpenMP not supported. Performance may be terrible.")
endif()

add_executable(compare-multiple src/compare_multiple.cpp)
target_link_libraries(compare-multiple PRIVATE mesh-diff)

add_executable(compare-single src/compare_single.cpp)
target_link_libraries(compare-single PRIVATE mesh-diff)

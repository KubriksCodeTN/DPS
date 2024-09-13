cmake_minimum_required(VERSION 3.16)
project(dps)
enable_language(CUDA)

option(DPS_BUILD_SEQUENTIAL "Build C++ version" ON)
option(DPS_BUILD_CUDA "Build CUDA version" OFF)
option(DPS_BUILD_TESTS "Build test suite" OFF)

add_subdirectory(src)

if(DPS_BUILD_TESTS)

    set(OMPL_BUILD_TESTS OFF CACHE BOOL "Build OMPL tests" )
    set(OMPL_BUILD_DEMOS OFF CACHE BOOL "Build OMPL demos" )
    set(OMPL_REGISTRATION "OFF" CACHE BOOL "Enable OMPL registration")
    add_subdirectory(external_libraries/ompl)
    set(OMPL_INCLUDE_DIRS ${CMAKE_BINARY_DIR}/external_libraries/ompl/src)

    add_subdirectory(external_libraries/mpdp)
    
    set(CLIPPER2_EXAMPLES OFF)
    set(CLIPPER2_TESTS OFF)
    add_subdirectory(external_libraries/clipper2/CPP)

    add_subdirectory(tests/dubins_ompl)
    add_subdirectory(tests/rrt)
    add_subdirectory(tests/mpdp)
    add_subdirectory(tests/roadmap)

endif()


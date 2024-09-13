cmake_minimum_required(VERSION 3.16)
project(dps_rrt)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Compiler options
set(COMPILE_OPTIONS -O3)

# find_package(boost REQUIRED)

add_executable(main_2d
    main_2d.cpp
    rrtstar.cpp
)
target_compile_options(main_2d PRIVATE ${COMPILE_OPTIONS})
target_include_directories(main_2d PRIVATE ${CMAKE_SOURCE_DIR}/external_libraries/nanoflann/include)
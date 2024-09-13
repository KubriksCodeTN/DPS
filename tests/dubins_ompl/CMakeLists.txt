# Set the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Compiler options
set(COMPILE_OPTIONS -O3)

# Include directories now only need Eigen3, OMPL includes are handled by FetchContent
set(INCLUDE_DIRS /usr/include/eigen3)

# Link libraries now only need boost_program_options, OMPL is linked directly
set(LINK_LIBS boost_program_options)

# Executable: dubins_seq_ompl
add_executable(dubins_seq_ompl 
    src/dubins_seq_ompl.cpp 
    ${CMAKE_SOURCE_DIR}/src/exec/main.cpp 
    ${CMAKE_SOURCE_DIR}/src/src/planner.cpp
)
target_include_directories(dubins_seq_ompl PRIVATE 
    ${INCLUDE_DIRS} 
    ${OMPL_INCLUDE_DIRS} 
    ${CMAKE_SOURCE_DIR}/src/src
)
target_link_libraries(dubins_seq_ompl PRIVATE ${LINK_LIBS} ompl dps)
target_compile_options(dubins_seq_ompl PRIVATE ${COMPILE_OPTIONS})

# Executable: rrt_dubins
add_executable(rrt_dubins 
    src/rrt_dubins.cpp
    src/bench_mr/Primitives.cpp 
)
target_include_directories(rrt_dubins PRIVATE ${INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS})
target_link_libraries(rrt_dubins PRIVATE ${LINK_LIBS} ompl)
target_compile_options(rrt_dubins PRIVATE ${COMPILE_OPTIONS})

# Executable: rrt
add_executable(rrt 
    src/rrt.cpp 
    src/bench_mr/Primitives.cpp
)
target_include_directories(rrt PRIVATE 
    ${INCLUDE_DIRS} 
    ${OMPL_INCLUDE_DIRS} 
    ${CMAKE_SOURCE_DIR}/src/src
)
target_link_libraries(rrt PRIVATE ${LINK_LIBS} ompl dps)
target_compile_options(rrt PRIVATE ${COMPILE_OPTIONS})

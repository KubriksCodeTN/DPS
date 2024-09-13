enable_language(CUDA)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CUDA_ARCHITECTURES "86")
# set(CMAKE_CUDA_COMPILER "/usr/local/cuda-12.6/bin/nvcc")

add_library(visilibity SHARED
    ${CMAKE_SOURCE_DIR}/external_libraries/visilibity/src/visilibity.cpp
)

set(dps_srcs "")
list(APPEND dps_srcs 
    src/planner.cpp
    src/multi_dubins/dubins.cpp
)

# Executable: dubins_cuda
if(DPS_BUILD_CUDA)
    add_library(dps_cuda SHARED
        ${dps_srcs}
        exec/dubins_cuda.cu
    )
    target_include_directories(dps_cuda
        PUBLIC src/
        PUBLIC ${CMAKE_SOURCE_DIR}/external_libraries/visilibity/src
    )
    target_link_libraries(dps_cuda PUBLIC visilibity)

    add_executable(dubins_cuda 
        exec/main.cpp 
    )
    target_link_libraries(dubins_cuda PRIVATE dps_cuda)
    target_compile_options(dubins_cuda PRIVATE ${COMPILE_OPTIONS})
    set_target_properties(dubins_cuda PROPERTIES CUDA_ARCHITECTURES ${CMAKE_CUDA_ARCHITECTURES})
endif()

# Executable: dubins_seq
if(DPS_BUILD_SEQUENTIAL)
    add_library(dps SHARED
        ${dps_srcs}
        exec/dubins_seq.cpp
    )
    target_include_directories(dps
        PUBLIC src/
        PUBLIC ${CMAKE_SOURCE_DIR}/external_libraries/visilibity/src
    )
    target_link_libraries(dps PUBLIC visilibity)

    add_executable(dubins_seq 
        exec/main.cpp 
    )
    target_link_libraries(dubins_seq PRIVATE dps)
    target_compile_options(dubins_seq PRIVATE ${COMPILE_OPTIONS})
endif()


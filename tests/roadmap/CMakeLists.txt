# Set the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Compiler options
set(COMPILE_OPTIONS -O3)

find_package(TBB REQUIRED)

include_directories(
    ../external_library/mpdp/srcCC/include
    ../external_library/mpdp/include
)

add_executable(roadmap
    src/main.cpp
    ${CMAKE_SOURCE_DIR}/src/src/multi_dubins/dubins.cpp
)
target_link_libraries(roadmap
    PRIVATE visilibity
    PRIVATE TBB::tbb
    PRIVATE Clipper2
    #PRIVATE Clipper2Utils
    PRIVATE MPDPCC
)

target_compile_options(roadmap PRIVATE ${COMPILE_OPTIONS})
target_include_directories(roadmap 
    PRIVATE ${CMAKE_SOURCE_DIR}/src/src/multi_dubins
    PRIVATE ${CMAKE_SOURCE_DIR}/external_libraries/clipper2/CPP/Clipper2Lib/include/clipper2
    PRIVATE ${CMAKE_SOURCE_DIR}/external_libraries/clipper2/CPP/Utils  
    PRIVATE ${CMAKE_SOURCE_DIR}/external_libraries/mpdp/srcCC/include
    PRIVATE ${CMAKE_SOURCE_DIR}/external_libraries/mpdp/include
    PRIVATE ${CMAKE_SOURCE_DIR}/external_libraries/visilibity/src
)
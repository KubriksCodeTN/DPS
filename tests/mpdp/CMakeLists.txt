# Set the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_executable(mpdp_test 
    main.cc
)

target_include_directories(mpdp_test
    PRIVATE ${CMAKE_SOURCE_DIR}/external_libraries/mpdp/srcCC/include
    PRIVATE ${CMAKE_SOURCE_DIR}/external_libraries/mpdp/include
)

target_link_libraries(mpdp_test PRIVATE MPDPCC)
target_compile_options(mpdp_test PRIVATE ${COMPILE_OPTIONS})
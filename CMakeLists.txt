cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(fyp_algo)

add_executable(fyp_algo
    fyp_algo.cpp
)

find_package(MAVSDK REQUIRED)

target_link_libraries(fyp_algo
    MAVSDK::mavsdk
)

if(NOT MSVC)
    add_compile_options(fyp_algo PRIVATE -Wall -Wextra)
else()
    add_compile_options(fyp_algo PRIVATE -WX -W2)
endif()
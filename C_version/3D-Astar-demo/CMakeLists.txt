cmake_minimum_required(VERSION 3.0)

project(code)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -g  -Wall -o2")
add_subdirectory(src)
include_directories(${CMAKE_SOURCE_DIR}/include)

add_executable(main main.cpp )
target_link_libraries(main SRC)
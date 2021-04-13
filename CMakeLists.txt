cmake_minimum_required(VERSION 3.5)
project(graph)

find_package(PNG REQUIRED)

add_compile_options("-g")

include_directories(${PNG_INCLUDE_DIR})
add_executable(graph main.cpp)
target_link_libraries(graph ${PNG_LIBRARY})
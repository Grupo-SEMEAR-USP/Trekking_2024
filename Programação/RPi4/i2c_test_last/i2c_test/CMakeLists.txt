cmake_minimum_required (VERSION 3.8)

project ("i2c_test")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")

add_subdirectory(bsc_i2c_controller)

add_executable (i2c.out
  exemplo1.cpp
)

include_directories("${CMAKE_CURRENT_SOURCE_DIR}")

target_link_libraries(i2c.out PUBLIC bsc_controller)
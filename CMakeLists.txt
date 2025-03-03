cmake_minimum_required(VERSION 2.8.3)
project(fiesta)
# 添加PROBABILISTIC定义
add_definitions(-DPROBABILISTIC)
include_directories(include)

link_directories(${PCL_LIBRARY_DIRS})

set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS} -O3 -Wall -pthread") # -Wextra -Werror
set(CMAKE_BUILD_TYPE "RELEASE")

add_library(
        ESDF
        src/parameters.cpp
        src/ESDFMap.cpp
)


add_executable(main
        main.cpp
        )
target_link_libraries(main ESDF)


cmake_minimum_required(VERSION 3.10)
project(rc_esdf_test)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")

find_package(Eigen3 REQUIRED)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})

include_directories(
   include    # 包含当前目录 (为了找到 rc_esdf.h)
    ${EIGEN3_INCLUDE_DIRS}    # 包含 Eigen 头文件
)

add_library(rc_esdf 
    src/rc_esdf.cpp
)
add_executable(test_rc_esdf 
    main.cpp 
)
target_link_libraries(test_rc_esdf rc_esdf ${OpenCV_LIBS})
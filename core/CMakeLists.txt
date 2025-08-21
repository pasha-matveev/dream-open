cmake_minimum_required(VERSION 3.28)

project(core)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(core src/main.cpp src/camera.cpp)

target_include_directories(core PRIVATE include)
target_link_libraries(core PRIVATE ${OpenCV_LIBS})
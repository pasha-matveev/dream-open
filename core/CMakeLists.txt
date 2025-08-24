cmake_minimum_required(VERSION 3.28)

project(core)

find_package( OpenCV REQUIRED )

find_library(LibSerial_LIBRARY Serial)

include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(core src/main.cpp src/camera.cpp src/tracking/object.cpp src/vec.cpp src/tracking/ball.cpp src/config.cpp src/robot.cpp)

target_include_directories(core PRIVATE include)
target_link_libraries(core PRIVATE ${OpenCV_LIBS} ${LibSerial_LIBRARY})
cmake_minimum_required(VERSION 3.25)

project(core)

find_package( OpenCV REQUIRED )

find_library(LibSerial_LIBRARY serial REQUIRED)

include_directories( ${OpenCV_INCLUDE_DIRS} )

aux_source_directory(src SRC_FILES)
aux_source_directory(src/tracking TRACKING_FILES)
aux_source_directory(src/utils UTILS_FILES)

add_executable(core ${SRC_FILES} ${TRACKING_FILES} ${UTILS_FILES})

target_include_directories(core PRIVATE include)
target_link_libraries(core PRIVATE ${OpenCV_LIBS} ${LibSerial_LIBRARY})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

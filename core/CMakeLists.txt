cmake_minimum_required(VERSION 3.18)

project(core)

find_package(OpenCV REQUIRED)
find_package(RapidJSON REQUIRED)

find_library(LibSerial_LIBRARY serial REQUIRED)
find_library(wiringPi_LIB wiringPi)

include_directories( ${OpenCV_INCLUDE_DIRS} )

aux_source_directory(src SRC_FILES)
aux_source_directory(src/tracking TRACKING_FILES)
aux_source_directory(src/utils UTILS_FILES)
aux_source_directory(src/gpio GPIO_FILES)
aux_source_directory(src/media MEDIA_FILES)

add_executable(core ${SRC_FILES} ${TRACKING_FILES} ${UTILS_FILES} ${GPIO_FILES} ${MEDIA_FILES})

target_include_directories(core PRIVATE include)
target_link_libraries(core PRIVATE ${OpenCV_LIBS} ${LibSerial_LIBRARY} )
target_link_libraries(core PRIVATE ${wiringPi_LIB})

target_precompile_headers(core PRIVATE
    <rapidjson/document.h>
    <rapidjson/istreamwrapper.h>
)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

cmake_minimum_required(VERSION 3.18)

project(core)

include_directories( ${OpenCV_INCLUDE_DIRS} )

aux_source_directory(src SRC_FILES)
aux_source_directory(src/tracking TRACKING_FILES)
aux_source_directory(src/utils UTILS_FILES)
aux_source_directory(src/gpio GPIO_FILES)
aux_source_directory(src/media MEDIA_FILES)

add_executable(core ${SRC_FILES} ${TRACKING_FILES} ${UTILS_FILES} ${GPIO_FILES} ${MEDIA_FILES})

target_include_directories(core PRIVATE include)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

# OpenCV
find_package(OpenCV REQUIRED)
target_link_libraries(core PRIVATE ${OpenCV_LIBS} )

# RapidJSON
find_package(RapidJSON REQUIRED)
target_precompile_headers(core PRIVATE
    <rapidjson/document.h>
    <rapidjson/istreamwrapper.h>
)

# LibSerial
find_library(LibSerial_LIB serial REQUIRED)
target_link_libraries(core PRIVATE ${LibSerial_LIB})

# WiringPi
find_library(WiringPi_LIB wiringPi REQUIRED)
target_link_libraries(core PRIVATE ${WiringPi_LIB})

# LibCamera
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAMERA REQUIRED IMPORTED_TARGET libcamera)
include_directories(${LIBCAMERA_INCLUDE_DIRS})
target_link_libraries(core PRIVATE PkgConfig::LIBCAMERA)

# spdlog
find_package(spdlog REQUIRED)
target_link_libraries(core PRIVATE spdlog::spdlog)
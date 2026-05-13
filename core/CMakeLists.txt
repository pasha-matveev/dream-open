cmake_minimum_required(VERSION 3.18)
# set (CMAKE_CXX_STANDARD 20)

project(core)

find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
    set(CMAKE_C_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
endif()
if(UNIX AND NOT APPLE)
    find_program(MOLD_LINKER mold)
    if(MOLD_LINKER)
        add_link_options("-fuse-ld=mold")
    endif()
endif()

include_directories( ${OpenCV_INCLUDE_DIRS} )

aux_source_directory(src SRC_FILES)
aux_source_directory(src/utils UTILS_FILES)
aux_source_directory(src/utils/geo GEO_FILES)
aux_source_directory(src/media MEDIA_FILES)
aux_source_directory(src/strategy STRATEGY_FILES)
aux_source_directory(src/strategy/common STRATEGY_COMMON_FILES)
aux_source_directory(src/strategy/roles STRATEGY_ROLES_FILES)
aux_source_directory(src/config CONFIG_FILES)
aux_source_directory(src/config/strategy CONFIG_STRATEGY_FILES)

if(APPLE)

set(TRACKING_FILES src/tracking/camera_mock.cpp src/tracking/object.cpp)
aux_source_directory(src/gpio_mock GPIO_FILES)

elseif(UNIX)

set(TRACKING_FILES src/tracking/camera.cpp src/tracking/object.cpp)
aux_source_directory(src/gpio GPIO_FILES)

endif()

add_executable(core ${SRC_FILES} ${TRACKING_FILES} ${UTILS_FILES} ${GEO_FILES} ${GPIO_FILES} ${MEDIA_FILES} ${STRATEGY_FILES} ${STRATEGY_COMMON_FILES} ${STRATEGY_ROLES_FILES} ${CONFIG_FILES} ${CONFIG_STRATEGY_FILES})

target_include_directories(core PRIVATE include)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

# OpenCV
find_package(OpenCV REQUIRED)

# RapidJSON
find_package(RapidJSON REQUIRED)
target_precompile_headers(core PRIVATE
    <rapidjson/document.h>
    <rapidjson/istreamwrapper.h>
)

# LibSerial
find_library(LibSerial_LIB serial REQUIRED)

# spdlog
find_package(spdlog REQUIRED)

# SFML

if(DEFINED ENV{FETCHCONTENT_BASE_DIR})
    set(FETCHCONTENT_BASE_DIR "$ENV{FETCHCONTENT_BASE_DIR}" CACHE PATH "")
endif()
include(FetchContent)
FetchContent_Declare(SFML
    GIT_REPOSITORY https://github.com/SFML/SFML.git
    GIT_TAG 3.0.2
    GIT_SHALLOW ON
    EXCLUDE_FROM_ALL
    SYSTEM)
FetchContent_MakeAvailable(SFML)

target_link_libraries(core PRIVATE ${OpenCV_LIBS} )
target_link_libraries(core PRIVATE ${LibSerial_LIB})
target_link_libraries(core PRIVATE SFML::Graphics)
target_link_libraries(core PRIVATE spdlog::spdlog)

if (UNIX AND NOT APPLE)

# WiringPi
find_library(WiringPi_LIB wiringPi REQUIRED)

# LCCV
find_library(LCCV_LIB lccv REQUIRED HINTS /usr/local/lib)

# LibCamera
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAMERA REQUIRED IMPORTED_TARGET libcamera)
include_directories(${LIBCAMERA_INCLUDE_DIRS})

target_link_libraries(core PRIVATE ${LCCV_LIB} ${WiringPi_LIB})
target_link_libraries(core PRIVATE PkgConfig::LIBCAMERA)

endif()



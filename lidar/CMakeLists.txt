cmake_minimum_required(VERSION 3.28)
project(lidar VERSION 0.1.0 LANGUAGES C CXX)

add_executable(main main.cpp)

include(FetchContent)
FetchContent_Declare(SFML
    GIT_REPOSITORY https://github.com/SFML/SFML.git
    GIT_TAG 3.0.0
    GIT_SHALLOW ON
    EXCLUDE_FROM_ALL
    SYSTEM)
FetchContent_MakeAvailable(SFML)

target_link_libraries(main PRIVATE SFML::Graphics)
target_include_directories(main PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/rplidar_sdk/sdk/include" "${CMAKE_CURRENT_SOURCE_DIR}/rplidar_sdk/sdk/src")
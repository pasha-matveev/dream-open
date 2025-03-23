cmake_minimum_required(VERSION 3.28)
project(CMakeSFMLProject LANGUAGES CXX)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# --------------------------------------------------
# Fetch SFML from GitHub (as you already do)
# --------------------------------------------------
include(FetchContent)
FetchContent_Declare(SFML
    GIT_REPOSITORY https://github.com/SFML/SFML.git
    GIT_TAG 3.0.0
    GIT_SHALLOW ON
    EXCLUDE_FROM_ALL
    SYSTEM)
FetchContent_MakeAvailable(SFML)

# --------------------------------------------------
# Detect Platform (Optional) & Set RPLIDAR Paths
# --------------------------------------------------
# Adjust these paths for each platform as needed.
# For example, on macOS if you built the SDK in output/Darwin/Release
# on Raspberry Pi if you built it in output/Linux/Release, etc.

set(RPLIDAR_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/rplidar_sdk")
set(RPLIDAR_INCLUDE_DIR "${RPLIDAR_ROOT}/sdk/include")

if(APPLE)
    message(STATUS "Detected macOS (Darwin).")
    set(RPLIDAR_LIB_DIR "${RPLIDAR_ROOT}/output/Darwin/Release/")
elseif(UNIX)
    message(STATUS "Detected Linux.")
    set(RPLIDAR_LIB_DIR "${RPLIDAR_ROOT}/output/Linux/Release")
else()
    message(WARNING "Unknown platform - using fallback RPLIDAR paths.")
    set(RPLIDAR_LIB_DIR "/path/to/rplidar/sdk/output/Unknown")
endif()

message(STATUS "RPLidar Include Path: ${RPLIDAR_INCLUDE_DIR}")
message(STATUS "RPLidar Libary Path: ${RPLIDAR_LIB_DIR}")

# --------------------------------------------------
# Add Your Executable
# --------------------------------------------------
add_executable(main main.cpp)
target_compile_features(main PRIVATE cxx_std_17)

# --------------------------------------------------
# Include & Link RPLIDAR
# --------------------------------------------------
target_include_directories(main PRIVATE
    ${RPLIDAR_INCLUDE_DIR}
)

# If you need to link to a custom library location
# (not system default), use link_directories:
# link_directories(${RPLIDAR_LIB_DIR})

# Link the RPLIDAR library (named libsl_lidar_sdk.a) as -lsl_lidar_sdk
# Also link pthread, because RPLIDAR uses pthread symbols.
target_link_libraries(main
    PRIVATE
        SFML::Graphics
        "${RPLIDAR_LIB_DIR}/libsl_lidar_sdk.a"
        pthread
)
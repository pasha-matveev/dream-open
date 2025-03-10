# --------------------------------------------------
# Detect Platform
# --------------------------------------------------
PLATFORM  ?= $(shell uname -s)
# PLATFORM now holds something like "Darwin" on macOS,
# or "Linux" on Raspberry Pi (Raspbian).

# Root location of your RPLIDAR SDK checkout
RPLIDAR_ROOT  = ${CURDIR}/rplidar_sdk

# Depending on PLATFORM, set the subfolder for the compiled RPLIDAR library
ifeq ($(PLATFORM), Darwin)
    RPLIDAR_LIB_DIR = $(RPLIDAR_ROOT)/output/Darwin/Release
else ifeq ($(PLATFORM), Linux)
    RPLIDAR_LIB_DIR = $(RPLIDAR_ROOT)/output/Linux/Release
endif

# --------------------------------------------------
# Compiler Settings
# --------------------------------------------------
CXX       = clang++  # or g++. For Raspberry Pi (Linux), g++ is common
CXXFLAGS  = -std=c++17 -Wall -Wextra

# --------------------------------------------------
# Include Paths
# --------------------------------------------------
# Adjust SFML includes on each platform if necessary.
# For example, on macOS with Homebrew on Apple Silicon:
#   -I/opt/homebrew/include
# On Raspberry Pi, maybe:
#   -I/usr/include
#
# For RPLIDAR, the headers live in $(RPLIDAR_ROOT)/include plus subdirs.

INCLUDES  = -I$(RPLIDAR_ROOT)/sdk/include \
            -I${CURDIR}/SFML/include

# --------------------------------------------------
# Library Paths
# --------------------------------------------------
# Add SFML library folder + RPLIDAR library folder.
# On Raspberry Pi, you might use /usr/lib for SFML or similar.

LIBPATHS  = -L$(RPLIDAR_LIB_DIR) \
            -L${CURDIR}/SFML/src

# --------------------------------------------------
# Libraries
# --------------------------------------------------
# SFML linking
SFML_LIBS      = -lsfml-graphics -lsfml-window -lsfml-system

# RPLIDAR library
# If your file is named "libsl_lidar_sdk.a", link with "-lsl_lidar_sdk".
RPLIDAR_LIBS   = -lsl_lidar_sdk

# Combine them
LIBS           = $(SFML_LIBS) $(RPLIDAR_LIBS)

# --------------------------------------------------
# Target / Sources
# --------------------------------------------------
TARGET    = main
SRCS      = main.cpp
OBJS      = $(SRCS:.cpp=.o)

# --------------------------------------------------
# Rules
# --------------------------------------------------
.PHONY: all clean info

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) $(LIBPATHS) -o $@ $^ $(LIBS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

clean:
	rm -f $(TARGET) $(OBJS)

# Optional: a debug target that just shows info
info:
	@echo "Platform detected: $(PLATFORM)"
	@echo "RPLIDAR library dir: $(RPLIDAR_LIB_DIR)"
	@echo "INCLUDES: $(INCLUDES)"
	@echo "LIBPATHS: $(LIBPATHS)"

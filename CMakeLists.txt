cmake_minimum_required(VERSION 3.10) # Specify the minimum version of CMake

# Set the project name and version
project(WiFiDistanceCalculator VERSION 1.0)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add include directories
include_directories(src/include)

# Add executable
add_executable(WiFiDistanceCalculator src/main.cpp src/WiFiScanner.cpp src/DistanceCalculator.cpp)

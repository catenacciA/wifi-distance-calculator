cmake_minimum_required(VERSION 3.10) # Set the minimum required version of CMake

project(WiFiDistanceCalculator) # Define the project name

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# For macOS, to use CoreWLAN, ensure we're compiling for macOS and link against the CoreWLAN framework
if(APPLE)
    find_library(COREWLAN_LIBRARY CoreWLAN)
    find_library(FOUNDATION_LIBRARY Foundation)
endif()

# Include directories
include_directories(${PROJECT_SOURCE_DIR}/src/include)

# Define the source files
set(SOURCE_FILES
    src/main.cpp
    src/WiFiDistanceCalculator.cpp
    src/platform/mac/MacOSWiFiScanner.mm # Use .mm for Objective-C++
)

# Add the executable based on the specified source files
add_executable(${PROJECT_NAME} ${SOURCE_FILES})

# Link libraries
if(APPLE)
    target_link_libraries(${PROJECT_NAME} ${COREWLAN_LIBRARY} ${FOUNDATION_LIBRARY})
endif()

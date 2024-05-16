cmake_minimum_required(VERSION 3.10)

project(WiFiScannerExample)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_VERBOSE_MAKEFILE OFF)

set(CMAKE_TOOLCHAIN_FILE "/opt/vcpkg/scripts/buildsystems/vcpkg.cmake" CACHE STRING "Vcpkg toolchain file")

include_directories(include)

set(DEMO_MAIN_SOURCES
    src/demo_main.cpp
    src/WiFiScanner.cpp
    src/WiFiScannerFactory.cpp
    src/DistanceCalculatorFactory.cpp
    src/LogDistanceCalculator.cpp
    lib/wifi_scan.c
)

set(OFFLINE_SOURCES
    src/offline_main.cpp
    src/WiFiScanner.cpp
    src/WiFiScannerFactory.cpp
    src/DistanceCalculatorFactory.cpp
    src/LogDistanceCalculator.cpp
    lib/wifi_scan.c
)

set(ONLINE_SOURCES
    src/online_main.cpp
    src/WiFiScanner.cpp
    src/WiFiScannerFactory.cpp
    src/DistanceCalculatorFactory.cpp
    src/LogDistanceCalculator.cpp
    src/FingerprintDatabase.cpp
    src/FingerprintMatcher.cpp
    src/LocationEstimator.cpp
    lib/wifi_scan.c
)

find_package(jsoncpp CONFIG REQUIRED)

add_executable(wifi_scanner_demo ${DEMO_MAIN_SOURCES})
target_link_libraries(wifi_scanner_demo PRIVATE JsonCpp::JsonCpp mnl)

add_executable(wifi_scanner_offline ${OFFLINE_SOURCES})
target_link_libraries(wifi_scanner_offline PRIVATE JsonCpp::JsonCpp mnl)

add_executable(wifi_scanner_online ${ONLINE_SOURCES})
target_link_libraries(wifi_scanner_online PRIVATE JsonCpp::JsonCpp mnl)

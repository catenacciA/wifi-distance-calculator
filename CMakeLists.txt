cmake_minimum_required(VERSION 3.10)

project(WiFiScannerExample)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_VERBOSE_MAKEFILE OFF)

set(CMAKE_TOOLCHAIN_FILE "/opt/vcpkg/scripts/buildsystems/vcpkg.cmake" CACHE STRING "Vcpkg toolchain file")

include_directories(include)

set(DEMO_MAIN_SOURCES
    src/main/demo_main.cpp
    src/wifi/WiFiScanner.cpp
    src/wifi/WiFiScannerFactory.cpp
    src/wifi/WiFiScannerWithConfig.cpp
    src/distance/DistanceCalculatorFactory.cpp
    src/distance/LogDistanceCalculator.cpp
    lib/wifi_scan.c
)

set(OFFLINE_SOURCES
    src/main/offline_main.cpp
    src/wifi/WiFiScanner.cpp
    src/wifi/WiFiScannerFactory.cpp
    src/wifi/WiFiScannerWithConfig.cpp
    src/distance/DistanceCalculatorFactory.cpp
    src/distance/LogDistanceCalculator.cpp
    lib/wifi_scan.c
)

set(ONLINE_SOURCES
    src/main/online_main.cpp
    src/wifi/WiFiScanner.cpp
    src/wifi/WiFiScannerFactory.cpp
    src/wifi/WiFiScannerWithConfig.cpp
    src/distance/DistanceCalculatorFactory.cpp
    src/distance/LogDistanceCalculator.cpp
    src/fingerprint/FingerprintDatabase.cpp
    src/fingerprint/FingerprintMatcher.cpp
    src/fingerprint/LocationEstimator.cpp
    lib/wifi_scan.c
)

set(CONFIG_TEST_SOURCES
    src/main/config_test.cpp
    src/wifi/WiFiScanner.cpp
    src/wifi/WiFiScannerFactory.cpp
    src/wifi/WiFiScannerWithConfig.cpp
    src/distance/DistanceCalculatorFactory.cpp
    src/distance/LogDistanceCalculator.cpp
    lib/wifi_scan.c
)

find_package(jsoncpp CONFIG REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(wifi_scanner_demo ${DEMO_MAIN_SOURCES})
target_link_libraries(wifi_scanner_demo PRIVATE JsonCpp::JsonCpp mnl Eigen3::Eigen)

add_executable(wifi_scanner_offline ${OFFLINE_SOURCES})
target_link_libraries(wifi_scanner_offline PRIVATE JsonCpp::JsonCpp mnl Eigen3::Eigen)

add_executable(wifi_scanner_online ${ONLINE_SOURCES})
target_link_libraries(wifi_scanner_online PRIVATE JsonCpp::JsonCpp mnl Eigen3::Eigen)

add_executable(wifi_scanner_with_config ${CONFIG_TEST_SOURCES})
target_link_libraries(wifi_scanner_with_config PRIVATE JsonCpp::JsonCpp mnl Eigen3::Eigen)

cmake_minimum_required(VERSION 3.10)

project(WiFiScannerExample)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_VERBOSE_MAKEFILE OFF)

include_directories(include)

option(BUILD_DEMO "Build the demo executable" OFF)
option(BUILD_OFFLINE "Build the offline executable" OFF)
option(BUILD_ONLINE "Build the online executable" OFF)
option(BUILD_CONFIG_TEST "Build the config test executable" OFF)
option(BUILD_OPTIMIZE "Build the optimize executable" OFF)
option(BUILD_FINGERPRINT_OPTIMIZE "Build the fingerprint optimize executable" ON)

set(DEMO_MAIN_SOURCES
    src/main/demo_main.cpp
    src/wifi/WiFiScanner.cpp
    src/distance/LogDistanceCalculator.cpp
    src/utility/ConfigParser.cpp
    lib/wifi_scan.c
)

set(OFFLINE_SOURCES
    src/main/offline_main.cpp
    src/wifi/WiFiScanner.cpp
    src/distance/LogDistanceCalculator.cpp
    src/utility/Logger.cpp
    src/utility/ProgressBar.cpp
    src/utility/ConfigParser.cpp
    lib/wifi_scan.c
)

set(ONLINE_SOURCES
    src/main/online_main.cpp
    src/wifi/WiFiScanner.cpp
    src/distance/LogDistanceCalculator.cpp
    src/fingerprint/FingerprintDatabase.cpp
    src/fingerprint/FingerprintMatcher.cpp
    src/fingerprint/LocationEstimator.cpp
    src/fingerprint/RSSISimilarityCalculator.cpp
    src/fingerprint/CSVDatabaseParser.cpp
    src/utility/ConfigParser.cpp
    lib/wifi_scan.c
)

set(CONFIG_TEST_SOURCES
    src/main/config_test.cpp
    src/wifi/WiFiScanner.cpp
    src/distance/LogDistanceCalculator.cpp
    src/utility/ConfigParser.cpp
    lib/wifi_scan.c
)

set(OPTIMIZE_MAIN_SOURCES
    src/main/optimize_main.cpp
    src/wifi/WiFiScanner.cpp
    src/distance/LogDistanceCalculator.cpp
    src/optimize/PositionEstimator.cpp
    "include/optimize/Distance3DFunctor.h"
    src/utility/ProgressBar.cpp
    src/utility/ConfigParser.cpp
    src/utility/MathUtils.cpp   
    lib/wifi_scan.c
)

set(FINGERPRINT_OPTIMIZE_SOURCES
    src/main/fingerprint_optimize_main.cpp
    src/optimize/FingerprintPositionEstimator.cpp
    src/fingerprint/FingerprintDatabase.cpp
    src/fingerprint/FingerprintMatcher.cpp
    src/fingerprint/LocationEstimator.cpp
    src/fingerprint/RSSISimilarityCalculator.cpp
    src/fingerprint/CSVDatabaseParser.cpp
)

find_package(jsoncpp CONFIG REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(GTSAM CONFIG REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

if(BUILD_DEMO)
    add_executable(wifi_scanner_demo ${DEMO_MAIN_SOURCES})
    target_link_libraries(wifi_scanner_demo PRIVATE JsonCpp::JsonCpp mnl Eigen3::Eigen gtsam)
endif()

if(BUILD_OFFLINE)
    add_executable(wifi_scanner_offline ${OFFLINE_SOURCES})
    target_link_libraries(wifi_scanner_offline PRIVATE JsonCpp::JsonCpp mnl Eigen3::Eigen gtsam)
endif()

if(BUILD_ONLINE)
    add_executable(wifi_scanner_online ${ONLINE_SOURCES})
    target_link_libraries(wifi_scanner_online PRIVATE JsonCpp::JsonCpp mnl Eigen3::Eigen gtsam)
endif()

if(BUILD_CONFIG_TEST)
    add_executable(wifi_scanner_with_config ${CONFIG_TEST_SOURCES})
    target_link_libraries(wifi_scanner_with_config PRIVATE JsonCpp::JsonCpp mnl Eigen3::Eigen gtsam)
endif()

if(BUILD_OPTIMIZE)
    add_executable(wifi_scanner_optimize ${OPTIMIZE_MAIN_SOURCES})
    target_link_libraries(wifi_scanner_optimize PRIVATE JsonCpp::JsonCpp mnl Eigen3::Eigen gtsam)
endif()

if(BUILD_FINGERPRINT_OPTIMIZE)
    add_executable(wifi_scanner_fingerprint_optimize ${FINGERPRINT_OPTIMIZE_SOURCES})
    target_link_libraries(wifi_scanner_fingerprint_optimize PRIVATE JsonCpp::JsonCpp Eigen3::Eigen gtsam)
endif()

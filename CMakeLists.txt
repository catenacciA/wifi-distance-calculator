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
    src/optimize/DistanceFunctor.cpp
    src/utility/ProgressBar.cpp
    src/utility/ConfigParser.cpp
    src/utility/MathUtils.cpp   
    lib/wifi_scan.c
)

find_package(jsoncpp CONFIG REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(GTSAM CONFIG REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(wifi_scanner_demo ${DEMO_MAIN_SOURCES})
target_link_libraries(wifi_scanner_demo PRIVATE JsonCpp::JsonCpp mnl Eigen3::Eigen gtsam)

add_executable(wifi_scanner_offline ${OFFLINE_SOURCES})
target_link_libraries(wifi_scanner_offline PRIVATE JsonCpp::JsonCpp mnl Eigen3::Eigen gtsam)

add_executable(wifi_scanner_online ${ONLINE_SOURCES})
target_link_libraries(wifi_scanner_online PRIVATE JsonCpp::JsonCpp mnl Eigen3::Eigen gtsam)

add_executable(wifi_scanner_with_config ${CONFIG_TEST_SOURCES})
target_link_libraries(wifi_scanner_with_config PRIVATE JsonCpp::JsonCpp mnl Eigen3::Eigen gtsam)

# Add new executable for optimize_main
add_executable(wifi_scanner_optimize ${OPTIMIZE_MAIN_SOURCES})
target_link_libraries(wifi_scanner_optimize PRIVATE JsonCpp::JsonCpp mnl Eigen3::Eigen gtsam)

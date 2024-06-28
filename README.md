# Repository Overview

This repository contains various executable and related projects. The primary focus is on the FingerprintPositionEstimator located in the `optimize` directory within the `src` directory. The main file for this project is `fingerprint_optimize_main.cpp`, which is the only component you need to compile and execute. In CMake, you can specifically compile the `BUILD_FINGERPRINT_OPTIMIZE` option, as it is the only one enabled.

## Getting Started

### Prerequisites
- **C++ Compiler**: Ensure a C++ compiler (e.g., g++, clang) is installed.
- **CMake**: Ensure CMake is installed.
- **Python**: Required for auxiliary scripts. Ensure Python 3.x is installed.
- **vcpkg**: For managing C++ dependencies.
- **eigne3**: For linear algebra operations.
- **gtsam**: For factor graph optimization.
- **jsoncpp**: For JSON parsing.

### Installation

1. **Clone the Repository**
   ```sh
   git clone https://github.com/catenacciA/wifi-distance-calculator.git
   cd wifi-distance-calculator
   ```

2. **Install vcpkg**
   ```sh
   git clone https://github.com/Microsoft/vcpkg.git /usr/local/vcpkg
   cd /usr/local/vcpkg
   ./bootstrap-vcpkg.sh
   ./vcpkg integrate install
   ```

3. **Install Dependencies via vcpkg**
   ```sh
   /usr/local/vcpkg/vcpkg install jsoncpp
   /usr/local/vcpkg/vcpkg install gtsam
   /usr/local/vcpkg/vcpkg install eigen3
   ```

4. **Set Up the Build System**
   Create a build directory and configure the project using CMake, specifying the vcpkg toolchain file:
   ```sh
   cd ~/wifi-distance-calculator
   mkdir build
   cd build
   cmake -DCMAKE_TOOLCHAIN_FILE=/usr/local/vcpkg/scripts/buildsystems/vcpkg.cmake ..
   make
   ```

### Usage

To run the FingerprintPositionEstimator:
```sh
cd build
./wifi_scanner_fingerprint_optimize ../data/output/map/wifi_fingerprinting_dataset.csv ../data/output/map/ap_positions_ordered.csv ../current_RSSI_Dataset.csv > output.txt
```
Where:
- `<fingerprintFile>`: Located in `data/output/map` directory.
- `<apFile>`: Located in `data/output/map` directory.
- `<currentRSSIFile>`: Located in the root directory.

### Build Configuration

The components needed to compile the FingerprintPositionEstimator are specified in CMake as follows:

```cmake
set(FINGERPRINT_OPTIMIZE_SOURCES
    src/main/fingerprint_optimize_main.cpp
    src/optimize/FingerprintPositionEstimator.cpp
    src/fingerprint/FingerprintDatabase.cpp
    src/fingerprint/FingerprintMatcher.cpp
    src/fingerprint/LocationEstimator.cpp
    src/fingerprint/RSSISimilarityCalculator.cpp
    src/fingerprint/CSVDatabaseParser.cpp
)
```

### Acknowledgements
This project includes the following libraries:
- [wifi-scan](https://github.com/bmegli/wifi-scan): C/C++ library for scanning WiFi signals.
- [jsoncpp](https://github.com/open-source-parsers/jsoncpp): JSON parser for C++.
- [gtsam](https://github.com/borglab/gtsam): Library for factor graph optimization.

### Contributing
Contributions are welcome! Please follow these steps:
1. Fork the repository.
2. Create your feature branch (`git checkout -b feature/AmazingFeature`).
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`).
4. Push to the branch (`git push origin feature/AmazingFeature`).
5. Open a Pull Request.

### License
This project is licensed under the GNU General Public License v3.0. See the [LICENSE](LICENSE) file for details.

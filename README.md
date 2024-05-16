# WiFi Distance Calculator

## Overview
WiFi Distance Calculator is a tool designed to estimate the distance between WiFi access points and devices using the Log-distance path loss model and the Fingerprinting Similarity Measure. The project is implemented in C++ and uses the `wifi-scan` library to scan WiFi signals. The tool can be used in offline mode to collect fingerprinting data and in online mode to estimate the location of a device based on the collected data.

## Features
- **Distance Calculation**: Estimates the distance using log-distance path loss models.
- **Fingerprinting**: Utilizes a fingerprint database to estimate location by creating a grid-based reference system.


## Getting Started

### Prerequisites
- **C++ Compiler**: Ensure you have a C++ compiler installed (e.g., g++, clang).
- **CMake**: Make sure CMake is installed.
- **Python**: Required for some auxiliary scripts. Ensure Python 3.x is installed.
- **vcpkg**: Required for managing C++ dependencies. Ensure it is installed in the `/opt` directory.
- **jsoncpp**: Install the jsoncpp library using vcpkg.

### Installation

1. **Clone the Repository**
   ```sh
   git clone https://github.com/catenacciA/wifi-distance-calculator.git
   cd wifi-distance-calculator
   ```

2. **Install Dependencies**
   - **C++ (jsoncpp and vcpkg)**:
       ```sh
        cd /opt
        git clone https://github.com/Microsoft/vcpkg.git
        cd vcpkg
        ./bootstrap-vcpkg.sh
        ./vcpkg integrate install
        ./vcpkg install jsoncpp
      ```
   - **Python Libraries (Optional)**: Install the required Python packages.
     ```sh
     pip install -r requirements.txt
     ```

3. **Build the Project**
   ```sh
   mkdir build
   cd build
   cmake ..
   make
   ```

### Configuration
- **Grid Configuration**: Modify the `grid_config.json` to set up the grid layout for location estimation. You would also need to specify the APs SSID's you are working with, you can, do it via the `targetSSIDs` label. Example:
  
  ```json
  {
      "grid_size": [9, 7],
      "inaccessible_points": [
          [1,0], [2,0], [3,0], [4,0], [5,0], [6,0], [7,0], [8,0], [9,0],
          [6,1], [7,1], [8,1], [9,1],
          [8,6], [9,6],
          [8,7], [9,7]
      ],
      "reference_points": [
          [1,1], [2,1], [3,1], [4,1], [5,1],
          [1,2], [2,2], [3,2], [4,2], [5,2], [6,2], [7,2], [8,2], [9,2],
          [1,3], [2,3], [3,3], [4,3], [5,3], [6,3], [7,3], [8,3], [9,3],
          [1,4], [2,4], [3,4], [4,4], [5,4], [6,4], [7,4], [8,4], [9,4],
          [1,5], [2,5], [3,5], [4,5], [5,5], [6,5], [7,5], [8,5], [9,5],
          [1,6], [2,6], [3,6], [4,6], [5,6], [6,6], [7,6],
          [1,7], [2,7], [3,7], [4,7], [5,7], [6,7], [7,7]
      ],
      "targetSSIDs": ["AP1", "AP2", "AP3"]
  }
  ```

### Fingerprinting
Fingerprinting works by creating a grid in your environment of choice and specifying the grid details in `grid_config.json`. The offline mode is used for acquiring fingerprinting data based on the JSON file. To do this, the user must run the offline executable and walk through the environment to collect data for all cells in the grid, which will then be saved in the CSV file.

## Usage

### Running in Offline Mode
To run the offline fingerprinting data collection:
```sh
cd build
sudo ./wifi_scanner_offline <WiFi_Interface> <Config_File> <Fingerprint_Data_File>
```
Where:
- `<WiFi_Interface>`: The WiFi interface to use for scanning.
- `<Config_File>`: The JSON file containing the grid configuration.
- `<Fingerprint_Data_File>`: The desired CSV file to save the fingerprint data.

To run the offline distance calculation:
```sh
cd build
sudo ./wifi_scanner_demo <WiFi_Interface> <Config_File>
```

### Running in Online Mode
For live location estimation:
```sh
cd build
sudo ./wifi_scanner_online <WiFi_Interface> <Config_File> <Fingerprint_Data_File>
```
Where:
- `<WiFi_Interface>`: The WiFi interface to use for scanning.
- `<Config_File>`: The JSON file containing the grid configuration.
- `<Fingerprint_Data_File>`: The CSV file containing the fingerprint data precently collected.

### Data Files
In the `data` directory, you can find various csv files that i collected from my own environment. Also several python scripts that i used to process the data to have a general understanding of this program's accuracy in estimating distances and locations.

## Acknowledgements
This project includes the following libraries:
- [wifi-scan](https://github.com/bmegli/wifi-scan): C/C++ library for scanning WiFi signals.
- [jsoncpp](https://github.com/open-source-parsers/jsoncpp): JSON parser for C++.

## Contributing
Contributions are welcome! Please follow these steps:

1. Fork the repository.
2. Create your feature branch (`git checkout -b feature/AmazingFeature`).
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`).
4. Push to the branch (`git push origin feature/AmazingFeature`).
5. Open a Pull Request.

## License
This project is licensed under the the GNU General Public License v3.0 License - see the [LICENSE](LICENSE) file for details.

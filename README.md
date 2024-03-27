# WiFi Distance Calculator

The WiFi Distance Calculator is a cross-platform C++ application that estimates the distance between the user and WiFi access points (APs) based on the signal strength (RSSI) of the WiFi network. This project demonstrates the use of an abstraction layer to provide platform-independent functionality alongside platform-specific implementations for Linux, Windows, and macOS.

## Features

- Scan for WiFi networks and display their SSIDs.
- Calculate the distance from the WiFi access points based on RSSI.
- Cross-platform support for Linux, Windows, and macOS.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites

- C++ compiler (GCC for Linux, MSVC for Windows, Clang for macOS)
- CMake (Version 3.15 or higher)
- Git

### Installing

1. Clone the repository:
```bash
git clone https://github.com/your_username/wifi-distance-calculator.git
```

2. Navigate to the project directory:
```bash
cd wifi-distance-calculator
```

3. Build the project with CMake:
```bash
mkdir build && cd build
cmake ..
make
```

4. Run the application (the executable name might differ based on your platform):
```bash
./WifiDistanceCalculator
```

## Usage

After launching the application, it will automatically begin scanning for WiFi networks and calculating distances based on RSSI values. The results will be displayed in the terminal.

## Contributing

Please read [CONTRIBUTING.md](CONTRIBUTING.md) for details on our code of conduct, and the process for submitting pull requests to us.

## Versioning

We use [SemVer](http://semver.org/) for versioning. For the versions available, see the [tags on this repository](https://github.com/catenacciA/wifi-distance-calculator/tags).

## Authors

- **Alessandro Catenacci** - *Initial work* - [catenacciA](https://github.com/catenacciA)

See also the list of [contributors](https://github.com/catenacciA/wifi-distance-calculator/contributors) who participated in this project.

## License

This project is licensed under the GNU General Public License v3.0 - see the [LICENSE.md](LICENSE.md) file for details.

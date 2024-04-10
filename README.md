# WifiDistanceCalculator

## Overview
WifiDistanceCalculator is a C++ project designed to run on Linux machines. It calculates the distance from the computer to all visible Access Points (APs) based on the path loss model formula. This application uses a dedicated thread to scan and update distances in real-time, providing a dynamic view of the wifi landscape around the computer.

## Path Loss Model Formula
The distance calculation is based on the following path loss model formula:

```math
L_{(dB)} = L_0 + 10 \cdot n \cdot \log_10(d/d_0) + X_σ
```

- $L_{(dB)}$ is the path loss in decibels (dB).
- $L_0$ is the path loss at a reference distance in dB.
- $n$ is the path loss exponent, indicating how fast the signal decays with distance.
- $d$ is the distance from the transmitter (in meters).
- $d_0$ is the reference distance (in meters).
- $X_σ$ is a variable accounting for the Gaussian random variable (in dB) representing the shadowing effect.

Using this formula, WifiDistanceCalculator estimates the distance to each AP, given the signal strength and other known constants.

## Prerequisites
- Linux operating system
- C++ compiler (g++ recommended)
- Make
- Wireless tools and libraries for Linux

## Installation

Clone the repository to your local machine:

```bash
git clone https://github.com/catenacciA/wifi-distance-calculator
cd WifiDistanceCalculator
```

Compile the project:

```bash
mkdir build && cd build
cmake ..
make
```

## Usage

To start the WifiDistanceCalculator, execute with root privileges (required for wireless scanning):

```bash
sudo ./WifiDistanceCalculator
```

The program will continuously scan for APs and update their distances in real-time. Output is displayed in the terminal window.

## Features
- Real-time scanning of visible Access Points (APs).
- Distance calculation using the path loss model formula.
- Multi-threading for continuous scanning and calculation.

## Contributing
Contributions to WifiDistanceCalculator are welcome. Please follow these steps:

1. Fork the repository.
2. Create your feature branch (`git checkout -b feature/AmazingFeature`).
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`).
4. Push to the branch (`git push origin feature/AmazingFeature`).
5. Open a Pull Request.

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments
- Thanks to all contributors who have helped in refining the WifiDistanceCalculator.
- Special thanks to the developers and maintainers of the wireless tools and libraries for Linux.
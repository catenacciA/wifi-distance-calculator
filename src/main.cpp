#include "include/platform/MacOSWiFiScanner.h"
#include <iostream>

int main() {
    MacOSWiFiScanner scanner;

    // Ensure the scanner is initialized correctly.
    if (!scanner.initialize()) {
        std::cerr << "Failed to initialize WiFi scanner." << std::endl;
        return 1;
    }

    // Scan for networks.
    auto networks = scanner.scanForNetworks();

    // Check if any networks were found.
    if (networks.empty()) {
        std::cout << "No WiFi networks found. Please ensure you're in range of a WiFi network." << std::endl;
        return 0; // Return successfully as this is not necessarily an error state.
    }

    // Proceed to list found networks.
    std::cout << "Found " << networks.size() << " networks:" << std::endl;
    for (const auto& network : networks) {
        // Print each network's SSID and RSSI. This assumes the scanForNetworks method ensures data integrity.
        std::cout << "SSID: " << network.ssid << ", RSSI: " << network.rssi << " dBm" << std::endl;
    }

    return 0;
}
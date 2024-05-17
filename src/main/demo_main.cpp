#include "../../include/distance/DistanceCalculatorFactory.h"
#include "../../include/wifi/WiFiScannerFactory.h"
#include <iostream>
#include <sstream>
#include <vector>

void printAPInfo(const std::vector<WiFiScanner::APInfo> &apInfos) {
  std::ostringstream output;
  for (const auto &apInfo : apInfos) {
    output << "BSSID: " << apInfo.bssid << ", SSID: " << apInfo.ssid
           << ", Signal: " << apInfo.signalStrength
           << " dBm, Distance: " << apInfo.distance << " meters" << std::endl;
  }
  std::cout << output.str();
}

void printUsage(const std::string &programName) {
  std::cout << "Usage: " << programName << " <WiFi_Interface> <Config_File> [-a]"
            << std::endl;
  std::cout << "  <WiFi_Interface>: The name of the WiFi interface to use "
               "(e.g., wlan0)"
            << std::endl;
  std::cout << "  <Config_File>: The path to the configuration file"
            << std::endl;
  std::cout << "  -a: Scan all visible APs (optional)" << std::endl;
}

int main(int argc, char *argv[]) {
  if (argc < 3) {
    printUsage(argv[0]);
    return 1;
  }

  bool scanAll = false;
  std::string wifiInterface = argv[1];
  std::string configFile = argv[2];

  if (argc > 3 && std::string(argv[3]) == "-a") {
    scanAll = true;
  }

  try {
    auto distanceCalculator =
        DistanceCalculatorFactory::createLogDistanceCalculator();
    auto wifiScanner = WiFiScannerFactory::create(
        wifiInterface, std::move(distanceCalculator), configFile);
    auto apInfos = wifiScanner->scan(!scanAll);
    printAPInfo(apInfos);
  } catch (const std::exception &e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
#include "../../include/distance/LogDistanceCalculator.h"
#include "../../include/utility/ConfigParser.h"
#include "../../include/wifi/WiFiScanner.h"
#include <iostream>
#include <sstream>
#include <vector>

void printAPInfo(const std::vector<IWiFiScanner::APInfo> &apInfos) {
  std::ostringstream output;
  for (const auto &apInfo : apInfos) {
    output << "BSSID: " << apInfo.bssid << ", SSID: " << apInfo.ssid
           << ", Signal: " << apInfo.signalStrength
           << " dBm, Distance: " << apInfo.distance << " meters" << std::endl;
  }
  std::cout << output.str();
}

void printUsage(const std::string &programName) {
  std::cout << "Usage: " << programName
            << " <WiFi_Interface> <Config_File> [-a]" << std::endl;
  std::cout << "  <WiFi_Interface>: The name of the WiFi interface to use "
            << "(e.g., wlan0)" << std::endl;
  std::cout << "  <Config_File>: The path to the configuration file (JSON)"
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
    // Create the ConfigParser instance
    std::shared_ptr<IConfigParser> configParser =
        std::make_shared<ConfigParser>(configFile);

    // Retrieve the AP parameters from the config
    const auto &apParameters = configParser->getAPParameters();

    // Create the distance calculator instance for each AP
    std::map<std::string, std::unique_ptr<IDistanceCalculator>>
        distanceCalculators;
    for (const auto &[bssid, params] : apParameters) {
      distanceCalculators[bssid] = std::make_unique<LogDistanceCalculator>(
          params.pathLossExponent, params.referenceDistance,
          params.referencePathLoss, params.transmitPower, params.sigma);
    }

    // Initialize the WiFi scanner with the map of distance calculators
    WiFiScanner wifiScanner(wifiInterface, configParser,
                            std::move(distanceCalculators));

    // Scan for APs
    auto apInfos = wifiScanner.scan(!scanAll);
    printAPInfo(apInfos);
  } catch (const std::exception &e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}

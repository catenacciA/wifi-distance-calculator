#include "../../include/distance/LogDistanceCalculator.h"
#include "../../include/utility/Logger.h"
#include "../../include/utility/ProgressBar.h"
#include "../../include/utility/ConfigParser.h"
#include "../../include/wifi/WiFiScanner.h"
#include <iostream>

int main() {
  std::string wifiInterface = "wlp1s0f0";
  std::string configFile = "../config.json";

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
        params.pathLossExponent, params.referenceDistance, params.referencePathLoss,
        params.transmitPower, params.sigma);
  }

  // Initialize the WiFi scanner with the map of distance calculators
  WiFiScanner wifiScanner(wifiInterface, configParser,
                          std::move(distanceCalculators));

  // Scan for APs
  auto apInfos = wifiScanner.scan(true);

  // Print the results
  const auto &apPositions = wifiScanner.getAPPositions();
  for (const auto &apInfo : apInfos) {
    auto it = apPositions.find(apInfo.bssid);
    if (it != apPositions.end()) {
      std::cout << "AP: " << apInfo.bssid << " Distance: " << apInfo.distance
                << " Position: " << it->second.transpose() << std::endl;
    } else {
      std::cout << "AP BSSID: " << apInfo.bssid
                << " Distance: " << apInfo.distance << " Position: ("
                << apInfo.position.x() << ", " << apInfo.position.y() << ", "
                << apInfo.position.z() << ")" << std::endl;
    }
  }

  return 0;
}

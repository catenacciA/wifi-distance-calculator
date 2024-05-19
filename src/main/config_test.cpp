#include "../../include/Logger.h"
#include "../../include/ProgressBar.h"
#include "../../include/distance/LogDistanceCalculator.h"
#include "../../include/wifi/WiFiScanner.h"
#include <iostream>

int main() {
  std::string wifiInterface = "wlp1s0f0";
  std::string configFile = "../config.json";

  // Create the distance calculator
  auto distanceCalculator =
      std::make_unique<LogDistanceCalculator>(14.61, 1, 41.72, 39.40);

  // Create the WiFi scanner with configuration
  WiFiScanner wifiScanner(wifiInterface, std::move(distanceCalculator),
                          configFile);

  // Scan for APs
  auto apInfos = wifiScanner.scan(true);

  // Print AP scan results with positions from config
  const auto &apPositions = wifiScanner.getAPPositions();
  for (const auto &apInfo : apInfos) {
    auto it = apPositions.find(apInfo.bssid);
    if (it != apPositions.end()) {
      std::cout << "AP: " << apInfo.bssid
                << " Distance: " << apInfo.distance
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

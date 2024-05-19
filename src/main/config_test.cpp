#include "../../include/distance/DistanceCalculatorFactory.h"
#include "../../include/wifi/WiFiScannerFactory.h"
#include "../../include/wifi/WiFiScannerWithConfig.h"
#include <iostream>

int main() {
  std::string wifiInterface = "wlp1s0f0";
  std::string configFile = "../grid_config.json";
  auto distanceCalculator =
      DistanceCalculatorFactory::createLogDistanceCalculator();

  auto wifiScannerWithConfig = WiFiScannerFactory::createWithConfig(
      wifiInterface, std::move(distanceCalculator), configFile);
  auto apInfos = wifiScannerWithConfig->scan(true);

  // Print AP scan results with positions from config
  const auto &apPositions = wifiScannerWithConfig->getAPPositions();
  for (const auto &apInfo : apInfos) {
    auto it = apPositions.find(apInfo.bssid);
    if (it != apPositions.end()) {
      std::cout << "AP BSSID: " << apInfo.bssid
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

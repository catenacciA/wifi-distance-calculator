#include "../../include/wifi/WiFiScannerWithConfig.h"
#include "../../include/wifi/WiFiScannerFactory.h"
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <json/json.h>

WiFiScannerWithConfig::WiFiScannerWithConfig(
    const std::string &interface,
    std::unique_ptr<DistanceCalculator> distanceCalculator,
    const std::string &configFile) {
  wifiScanner = WiFiScannerFactory::create(
      interface, std::move(distanceCalculator), configFile);
}

std::vector<WiFiScanner::APInfo> WiFiScannerWithConfig::scan(bool scanAll) {
  return wifiScanner->scan(scanAll);
}

const std::map<std::string, Eigen::Vector3d> &
WiFiScannerWithConfig::getAPPositions() const {
  return wifiScanner->getAPPositions();
}

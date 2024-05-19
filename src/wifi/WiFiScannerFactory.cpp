#include "../../include/wifi/WiFiScannerFactory.h"
#include "../../include/wifi/WiFiScanner.h"
#include "../../include/wifi/WiFiScannerWithConfig.h"
#include <memory>

std::unique_ptr<WiFiScanner> WiFiScannerFactory::create(
    const std::string &interface,
    std::unique_ptr<DistanceCalculator> distanceCalculator,
    const std::string &configFile) {
  return std::make_unique<WiFiScanner>(interface, std::move(distanceCalculator),
                                       configFile);
}

std::unique_ptr<WiFiScannerWithConfig> WiFiScannerFactory::createWithConfig(
    const std::string &interface,
    std::unique_ptr<DistanceCalculator> distanceCalculator,
    const std::string &configFile) {
  return std::make_unique<WiFiScannerWithConfig>(
      interface, std::move(distanceCalculator), configFile);
}

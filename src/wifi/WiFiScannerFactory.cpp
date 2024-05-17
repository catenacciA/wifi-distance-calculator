#include "../../include/wifi/WiFiScannerFactory.h"
#include "../../include/wifi/WiFiScanner.h"
#include <memory>

std::unique_ptr<WiFiScanner> WiFiScannerFactory::create(
    const std::string &interface,
    std::unique_ptr<DistanceCalculator> distanceCalculator,
    const std::string &configFile) {
  return std::make_unique<WiFiScanner>(interface,
                                       std::move(distanceCalculator),
                                       configFile);
}
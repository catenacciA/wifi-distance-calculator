#ifndef WIFISCANNERFACTORY_H
#define WIFISCANNERFACTORY_H

#include "../distance/DistanceCalculator.h"
#include "WiFiScanner.h"
#include "WiFiScannerWithConfig.h"
#include <memory>
#include <string>

// Factory class for creating WiFiScanner objects
class WiFiScannerFactory {
public:
  static std::unique_ptr<WiFiScanner>
  create(const std::string &interface,
         std::unique_ptr<DistanceCalculator> distanceCalculator,
         const std::string &configFile);

  static std::unique_ptr<WiFiScannerWithConfig>
  createWithConfig(const std::string &interface,
                   std::unique_ptr<DistanceCalculator> distanceCalculator,
                   const std::string &configFile);
};

#endif // WIFISCANNERFACTORY_H

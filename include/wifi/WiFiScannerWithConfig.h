#ifndef WIFISCANNERWITHCONFIG_H
#define WIFISCANNERWITHCONFIG_H

#include "../distance/DistanceCalculator.h"
#include "WiFiScanner.h"
#include <Eigen/Dense>
#include <json/json.h>
#include <map>
#include <memory>
#include <string>

class WiFiScannerWithConfig {
public:
  WiFiScannerWithConfig(const std::string &interface,
                        std::unique_ptr<DistanceCalculator> distanceCalculator,
                        const std::string &configFile);

  std::vector<WiFiScanner::APInfo> scan(bool scanAll);

  const std::map<std::string, Eigen::Vector3d> &getAPPositions() const;

private:
  void parseConfig(const std::string &configFile);

  std::unique_ptr<WiFiScanner> wifiScanner;
  std::map<std::string, Eigen::Vector3d> apPositions;
};

#endif // WIFISCANNERWITHCONFIG_H

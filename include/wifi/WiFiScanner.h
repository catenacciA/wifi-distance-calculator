#ifndef WIFISCANNER_H
#define WIFISCANNER_H

#include "../distance/DistanceCalculator.h"
#include "wifi_scan.h"
#include <memory>
#include <set>
#include <string>
#include <vector>
#include <map>
#include <Eigen/Dense>

// Class for scanning for WiFi access points
class WiFiScanner {
public:
  explicit WiFiScanner(const std::string &interface,
                       std::unique_ptr<DistanceCalculator> distanceCalculator,
                       const std::string &configFile);
  ~WiFiScanner();

  struct APInfo {
    std::string bssid;
    uint32_t frequency;
    std::string ssid;
    int32_t signalStrength;
    double distance;
    int32_t seenMsAgo;
    Eigen::Vector3d position;

    APInfo() : position(Eigen::Vector3d::Zero()) {}
  };

  std::vector<APInfo> scan(bool filter = true);
  void logData(const std::string &cellId, int numScans = 10,
               const std::string &filename = "");

  // Add a method to get AP positions
  const std::map<std::string, Eigen::Vector3d>& getAPPositions() const;

private:
  void parseConfig(const std::string &configFile);
  void updateAPPositions(std::vector<APInfo> &aps);

  struct wifi_scan *wifi;
  std::unique_ptr<DistanceCalculator> distanceCalculator;
  std::set<std::string> targetBSSIDs;
  std::map<std::string, Eigen::Vector3d> apPositions;
};

#endif // WIFISCANNER_H

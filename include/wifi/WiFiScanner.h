#ifndef WIFISCANNER_H
#define WIFISCANNER_H

#include "../distance/IDistanceCalculator.h"
#include "IWiFiScanner.h"
#include "wifi_scan.h"
#include <map>
#include <memory>
#include <set>
#include <string>

class WiFiScanner : public IWiFiScanner {
public:
  explicit WiFiScanner(const std::string &interface,
                       std::unique_ptr<IDistanceCalculator> distanceCalculator,
                       const std::string &configFile = "");
  ~WiFiScanner();

  std::vector<APInfo> scan(bool filter = true) override;
  const std::map<std::string, Eigen::Vector3d> &getAPPositions() const override;
  IDistanceCalculator *getDistanceCalculator() const;

private:
  void parseConfig(const std::string &configFile);

  struct wifi_scan *wifi;
  std::unique_ptr<IDistanceCalculator> distanceCalculator;
  std::set<std::string> targetBSSIDs;
  std::map<std::string, Eigen::Vector3d> apPositions;
};

#endif // WIFISCANNER_H

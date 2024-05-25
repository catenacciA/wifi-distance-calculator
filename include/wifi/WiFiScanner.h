#ifndef WIFISCANNER_H
#define WIFISCANNER_H

#include "../distance/IDistanceCalculator.h"
#include "../utility/IConfigParser.h"
#include "IWiFiScanner.h"
#include "wifi_scan.h"
#include <map>
#include <memory>
#include <set>
#include <string>

class WiFiScanner : public IWiFiScanner {
public:
  explicit WiFiScanner(
      const std::string &interface, std::shared_ptr<IConfigParser> config,
      std::map<std::string, std::unique_ptr<IDistanceCalculator>>
          distanceCalculators);
  ~WiFiScanner();

  std::vector<APInfo> scan(bool filter = true) override;
  const std::map<std::string, Eigen::Vector3d> &getAPPositions() const override;
  IDistanceCalculator *getDistanceCalculator(const std::string &bssid) const;

private:
  struct wifi_scan *wifi;
  std::map<std::string, std::unique_ptr<IDistanceCalculator>>
      distanceCalculators;
  std::set<std::string> targetBSSIDs;
  std::map<std::string, Eigen::Vector3d> apPositions;
};

#endif // WIFISCANNER_H

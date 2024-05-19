#ifndef I_WIFISCANNER_H
#define I_WIFISCANNER_H

#include <Eigen/Dense>
#include <map>
#include <string>
#include <vector>

class IWiFiScanner {
public:
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

  virtual std::vector<APInfo> scan(bool filter = true) = 0;
  virtual const std::map<std::string, Eigen::Vector3d> &
  getAPPositions() const = 0;

  virtual ~IWiFiScanner() = default;
};

#endif // I_WIFISCANNER_H

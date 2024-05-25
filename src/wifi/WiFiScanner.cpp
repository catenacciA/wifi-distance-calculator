#include "../../include/wifi/WiFiScanner.h"
#include <iostream>
#include <stdexcept>

WiFiScanner::WiFiScanner(
    const std::string &interface, std::shared_ptr<IConfigParser> config,
    std::map<std::string, std::unique_ptr<IDistanceCalculator>>
        distanceCalculators)
    : distanceCalculators(std::move(distanceCalculators)),
      targetBSSIDs(config->getTargetBSSIDs().begin(),
                   config->getTargetBSSIDs().end()),
      apPositions(config->getAPPositions()) {

  wifi = wifi_scan_init(interface.c_str());
  if (!wifi) {
    throw std::runtime_error("Failed to initialize WiFi scan");
  }
}

WiFiScanner::~WiFiScanner() { wifi_scan_close(wifi); }

IDistanceCalculator *
WiFiScanner::getDistanceCalculator(const std::string &bssid) const {
  auto it = distanceCalculators.find(bssid);
  if (it != distanceCalculators.end()) {
    return it->second.get();
  }
  return nullptr;
}

std::vector<IWiFiScanner::APInfo> WiFiScanner::scan(bool filter) {
  const int maxAPs = 100;
  struct bss_info bssInfos[maxAPs];
  int numAPs = wifi_scan_all(wifi, bssInfos, maxAPs);

  if (numAPs < 0) {
    throw std::runtime_error("Failed to scan WiFi networks");
  }

  std::vector<APInfo> apInfos;

  for (int i = 0; i < numAPs && i < maxAPs; ++i) {
    char bssidStr[BSSID_STRING_LENGTH];
    snprintf(bssidStr, BSSID_STRING_LENGTH, "%02x:%02x:%02x:%02x:%02x:%02x",
             bssInfos[i].bssid[0], bssInfos[i].bssid[1], bssInfos[i].bssid[2],
             bssInfos[i].bssid[3], bssInfos[i].bssid[4], bssInfos[i].bssid[5]);

    std::string bssid(bssidStr);
    if (!filter || targetBSSIDs.find(bssid) != targetBSSIDs.end()) {
      APInfo apInfo;
      apInfo.bssid = bssid;
      apInfo.ssid = std::string(bssInfos[i].ssid);
      apInfo.signalStrength = bssInfos[i].signal_mbm / 100;

      IDistanceCalculator *calculator = getDistanceCalculator(bssid);
      if (calculator) {
        apInfo.distance = calculator->calculateDistance(apInfo.signalStrength);
      } else {
        apInfo.distance = 0.0; // TODO: handle the error appropriately
      }

      apInfo.seenMsAgo = bssInfos[i].seen_ms_ago;

      auto it = apPositions.find(bssid);
      if (it != apPositions.end()) {
        apInfo.position = it->second;
      } else {
        apInfo.position = Eigen::Vector3d::Zero();
      }

      apInfos.push_back(apInfo);
    }
  }

  return apInfos;
}

const std::map<std::string, Eigen::Vector3d> &
WiFiScanner::getAPPositions() const {
  return apPositions;
}

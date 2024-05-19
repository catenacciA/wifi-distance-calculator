#include "../../include/wifi/WiFiScanner.h"
#include <fstream>
#include <iostream>
#include <json/json.h>
#include <stdexcept>

WiFiScanner::WiFiScanner(
    const std::string &interface,
    std::unique_ptr<IDistanceCalculator> distanceCalculator,
    const std::string &configFile)
    : distanceCalculator(std::move(distanceCalculator)) {
  wifi = wifi_scan_init(interface.c_str());
  if (!wifi) {
    throw std::runtime_error("Failed to initialize WiFi scan");
  }

  if (!configFile.empty()) {
    parseConfig(configFile);
  }
}

WiFiScanner::~WiFiScanner() { wifi_scan_close(wifi); }

void WiFiScanner::parseConfig(const std::string &configFile) {
  std::ifstream file(configFile, std::ifstream::binary);
  if (!file.is_open()) {
    throw std::runtime_error("Unable to open configuration file");
  }

  Json::Value root;
  file >> root;

  const Json::Value bssids = root["targetBSSIDs"];
  for (const auto &bssid : bssids) {
    targetBSSIDs.insert(bssid.asString());
  }

  const Json::Value positions = root["ap_positions"];
  for (const auto &bssid : positions.getMemberNames()) {
    const Json::Value &position = positions[bssid];
    apPositions[bssid] = Eigen::Vector3d(
        position[0].asDouble(), position[1].asDouble(), position[2].asDouble());
  }
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
      apInfo.distance =
          distanceCalculator->calculateDistance(apInfo.signalStrength);
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

IDistanceCalculator *WiFiScanner::getDistanceCalculator() const {
  return distanceCalculator.get();
}
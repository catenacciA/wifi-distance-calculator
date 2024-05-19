#include "../../include/wifi/WiFiScanner.h"
#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>
#include <json/json.h>
#include <map>
#include <numeric>
#include <set>
#include <thread>
#include <unordered_set>

void displayProgressBar(int current, int total) {
  int barWidth = 50;
  float progress = static_cast<float>(current) / total;
  int pos = static_cast<int>(barWidth * progress);

  std::cout << "[";
  for (int i = 0; i < barWidth; ++i) {
    if (i < pos)
      std::cout << "=";
    else if (i == pos)
      std::cout << ">";
    else
      std::cout << " ";
  }
  std::cout << "] " << int(progress * 100.0) << " %\r";
  std::cout.flush();
}

std::set<std::string> loadTargetBSSIDs(const std::string &filename) {
  std::ifstream file(filename, std::ifstream::binary);
  if (!file.is_open()) {
    throw std::runtime_error("Unable to open configuration file");
  }

  Json::Value root;
  file >> root;
  std::set<std::string> targetBSSIDs;

  const Json::Value bssids = root["targetBSSIDs"];
  for (const auto &bssid : bssids) {
    targetBSSIDs.insert(bssid.asString());
  }

  return targetBSSIDs;
}

std::map<std::string, Eigen::Vector3d> loadAPPositions(const std::string &filename) {
  std::ifstream file(filename, std::ifstream::binary);
  if (!file.is_open()) {
    throw std::runtime_error("Unable to open configuration file");
  }

  Json::Value root;
  file >> root;
  std::map<std::string, Eigen::Vector3d> apPositions;

  const Json::Value positions = root["ap_positions"];
  for (const auto &bssid : positions.getMemberNames()) {
    const Json::Value &position = positions[bssid];
    apPositions[bssid] = Eigen::Vector3d(
        position[0].asDouble(), position[1].asDouble(), position[2].asDouble());
  }

  return apPositions;
}

WiFiScanner::WiFiScanner(const std::string &interface,
                         std::unique_ptr<DistanceCalculator> distanceCalculator,
                         const std::string &configFile)
    : distanceCalculator(std::move(distanceCalculator)) {
  wifi = wifi_scan_init(interface.c_str());
  if (!wifi) {
    throw std::runtime_error("Failed to initialize WiFi scan");
  }
  targetBSSIDs = loadTargetBSSIDs(configFile);
  apPositions = loadAPPositions(configFile);
}

WiFiScanner::~WiFiScanner() { wifi_scan_close(wifi); }

std::vector<WiFiScanner::APInfo> WiFiScanner::scan(bool filter) {
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
             bssInfos[i].bssid[0], bssInfos[i].bssid[1],
             bssInfos[i].bssid[2], bssInfos[i].bssid[3],
             bssInfos[i].bssid[4], bssInfos[i].bssid[5]);

    std::string bssid(bssidStr);
    if (!filter || targetBSSIDs.find(bssid) != targetBSSIDs.end()) {
      APInfo apInfo;
      apInfo.bssid = bssid;
      apInfo.ssid = std::string(bssInfos[i].ssid);
      apInfo.signalStrength = bssInfos[i].signal_mbm / 100;
      apInfo.distance = distanceCalculator->calculateDistance(apInfo.signalStrength);
      apInfo.seenMsAgo = bssInfos[i].seen_ms_ago;
      
      // Set the AP position from the configuration, if available
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

const std::map<std::string, Eigen::Vector3d>& WiFiScanner::getAPPositions() const {
  return apPositions;
}

void WiFiScanner::logData(const std::string &cellId, int numScans,
                          const std::string &filename) {
  struct ScanData {
    std::string ssid;
    std::vector<int> rssiValues;
  };

  std::map<std::string, ScanData> rssiData;

  for (int i = 0; i < numScans; ++i) {
    try {
      auto apInfos = scan(true);
      for (const auto &apInfo : apInfos) {
        auto &data = rssiData[apInfo.bssid];
        data.ssid = apInfo.ssid;
        data.rssiValues.push_back(apInfo.signalStrength);
      }
      displayProgressBar(i + 1, numScans);
    } catch (const std::exception &e) {
      std::cerr << "Error during scan " << i + 1 << ": " << e.what()
                << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  std::cout << std::endl;

  std::ofstream logFile;
  bool fileExists = std::ifstream(filename).good();

  logFile.open(filename, std::ios_base::app);

  if (!fileExists) {
    logFile << "CellIDx, CellIDy, BSSID, SSID, AvgRSSI" << std::endl;
  }

  for (const auto &[bssid, data] : rssiData) {
    double avgRSSI =
        std::accumulate(data.rssiValues.begin(), data.rssiValues.end(), 0.0) /
        data.rssiValues.size();
    logFile << cellId << "," << bssid << "," << data.ssid << "," << avgRSSI
            << std::endl;
  }
  logFile.close();
}

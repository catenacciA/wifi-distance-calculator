#ifndef WIFISCANNER_H
#define WIFISCANNER_H

#include <vector>
#include <string>
#include <memory>
#include <set>
#include "wifi_scan.h"
#include "../distance/DistanceCalculator.h"

// Class for scanning for WiFi access points
class WiFiScanner {
public:
    explicit WiFiScanner(const std::string& interface, std::unique_ptr<DistanceCalculator> distanceCalculator, const std::string& configFile);
    ~WiFiScanner();

    struct APInfo {
        std::string bssid;
        uint32_t frequency;
        std::string ssid;
        int32_t signalStrength;
        double distance;
        int32_t seenMsAgo;
    };

    std::vector<APInfo> scan(bool filter = true);
    void logData(const std::string& cellId, int numScans = 10, const std::string &filename = "");

private:
    struct wifi_scan *wifi;
    std::unique_ptr<DistanceCalculator> distanceCalculator;
    std::set<std::string> targetSSIDs;
};

#endif // WIFISCANNER_H

#include "../../include/fingerprint/LocationEstimator.h"
#include "../../include/fingerprint/RSSIData.h"
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

// Constructor with WiFiScanner
LocationEstimator::LocationEstimator(
    std::unique_ptr<WiFiScanner> scanner,
    std::unique_ptr<FingerprintMatcher> matcher)
    : scanner(std::move(scanner)), matcher(std::move(matcher)) {}

// Constructor without WiFiScanner
LocationEstimator::LocationEstimator(
    std::unique_ptr<FingerprintMatcher> matcher)
    : matcher(std::move(matcher)) {}

std::vector<RSSIData>
LocationEstimator::readRSSIValuesFromFile(const std::string &filename) {
  std::vector<RSSIData> rssiValues;
  std::ifstream file(filename);

  if (!file.is_open()) {
    throw std::runtime_error("Failed to open RSSI values file: " + filename);
  }

  std::string line;
  // Skip the header line
  std::getline(file, line);
  while (std::getline(file, line)) {
    std::istringstream iss(line);
    RSSIData data;
    std::string token;

    std::getline(iss, data.macAddress, ',');
    std::getline(iss, token, ',');
    data.rssiValue = std::stod(token);
    std::getline(iss, token, ',');
    data.mean = std::stod(token);
    std::getline(iss, token, ',');
    data.std = std::stod(token);

    rssiValues.push_back(data);
  }

  file.close();

  return rssiValues;
}

std::string LocationEstimator::estimateLocation() {
  if (scanner) {
    auto apInfos = scanner->scan(true);
    std::vector<std::pair<std::string, int>> currentRSSIValues;
    for (const auto &apInfo : apInfos) {
      currentRSSIValues.emplace_back(apInfo.bssid, apInfo.signalStrength);
    }
    return matcher->match(currentRSSIValues);
  } else {
    throw std::runtime_error(
        "WiFiScanner is not available for location estimation");
  }
}

std::vector<std::pair<std::string, Eigen::Vector3d>>
LocationEstimator::estimateExtendedLocation(const std::string &rssiFile) {
  auto currentRSSIValues = readRSSIValuesFromFile(rssiFile);
  return matcher->matchExtended(currentRSSIValues);
}

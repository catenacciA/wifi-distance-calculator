#include "../../include/fingerprint/RSSISimilarityCalculator.h"
#include <iostream>

double RSSISimilarityCalculator::calculate(
    const std::map<std::string, double> &fingerprint,
    const std::vector<std::pair<std::string, int>> &currentRSSIValues) const {
  double similarity = 0.0;
  for (const auto &[bssid, rssi] : currentRSSIValues) {
    auto it = fingerprint.find(bssid);
    if (it != fingerprint.end()) {
      similarity += std::pow(it->second - rssi, 2);
    } else {
      similarity += std::pow(-100 - rssi, 2); // Penalize for missing AP
    }
  }
  return similarity;
}

double RSSISimilarityCalculator::calculate(
    const std::vector<IWiFiScanner::APInfo> &extendedFingerprint,
    const std::vector<RSSIData> &currentRSSIValues) const {

  std::cout << "Calculating similarity for extended fingerprint" << std::endl;
  double similarity = 0.0;

  // Print all currentRSSIValues for debugging
  std::cout << "Current RSSI Values:" << std::endl;
  for (const auto &currentRSSIData : currentRSSIValues) {
    std::cout << "BSSID: " << currentRSSIData.macAddress
              << " | RSSI: " << currentRSSIData.rssiValue
              << " | Mean: " << currentRSSIData.mean
              << " | Std: " << currentRSSIData.std << std::endl;
  }

  for (const auto &currentRSSIData : currentRSSIValues) {
    const std::string &bssid = currentRSSIData.macAddress;
    double mean = currentRSSIData.mean;
    double std = currentRSSIData.std;

    std::cout << "Processing BSSID: " << bssid 
              << " with mean RSSI: " << mean
              << " and std: " << std << std::endl;

    auto it =
        std::find_if(extendedFingerprint.begin(), extendedFingerprint.end(),
                     [&bssid](const IWiFiScanner::APInfo &apInfo) {
                       return apInfo.bssid == bssid;
                     });

    if (it != extendedFingerprint.end()) {
      double signalStrength = it->signalStrength;
      double diff = signalStrength - mean;
      double zScore = diff / std;
      similarity += std::pow(zScore, 2);
      std::cout << "Matched BSSID: " << bssid
                << " | Signal Strength: " << signalStrength
                << " | Mean: " << mean
                << " | Std: " << std
                << " | Difference: " << diff
                << " | Z-Score: " << zScore
                << " | Squared Z-Score: " << std::pow(zScore, 2) << std::endl;
    } else {
      double penalty = std::pow((-100 - mean) / std, 2);
      similarity += penalty; // Penalize for missing AP
      std::cout << "Missing BSSID: " << bssid
                << " | Mean: " << mean
                << " | Std: " << std
                << " | Penalty: " << penalty << std::endl;
    }
  }

  std::cout << "Total similarity: " << similarity << std::endl;
  return similarity;
}

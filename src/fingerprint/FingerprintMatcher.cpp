#include "../../include/fingerprint/FingerprintMatcher.h"
#include "../../include/fingerprint/RSSIData.h"
#include <iostream>
#include <set>

FingerprintMatcher::FingerprintMatcher(const FingerprintDatabase &db,
                                       ISimilarityCalculator &calc)
    : database(db), calculator(calc) {
  std::cout << "FingerprintMatcher initialized with database and calculator."
            << std::endl;
}

std::string FingerprintMatcher::match(
    const std::vector<std::pair<std::string, int>> &currentRSSIValues) const {
  std::cout << "Starting match method." << std::endl;
  const auto &db = database.getDatabase();
  std::string bestMatch;
  double bestSimilarity = std::numeric_limits<double>::max();

  for (const auto &[cellId, fingerprint] : db) {
    double similarity = calculator.calculate(fingerprint, currentRSSIValues);
    std::cout << "Calculated similarity for cellId: " << cellId << " is "
              << similarity << std::endl;
    if (similarity < bestSimilarity) {
      bestSimilarity = similarity;
      bestMatch = cellId;
      std::cout << "New best match found: " << bestMatch
                << " with similarity: " << bestSimilarity << std::endl;
    }
  }

  std::cout << "Best match: " << bestMatch
            << " with similarity: " << bestSimilarity << std::endl;
  return bestMatch;
}

std::vector<std::pair<std::string, Eigen::Vector3d>>
FingerprintMatcher::matchExtended(
    const std::vector<RSSIData> &currentRSSIValues) const {
  std::cout << "Starting matchExtended method." << std::endl;
  const auto &extendedDb = database.getExtendedDatabase();
  std::vector<std::pair<std::string, Eigen::Vector3d>> bestMatches;
  std::set<std::string> processedBssids;

  for (const auto &currentRSSIData : currentRSSIValues) {
    std::string currentBssid = currentRSSIData.macAddress;

    // Skip if this BSSID has already been processed
    if (processedBssids.find(currentBssid) != processedBssids.end()) {
      continue;
    }

    double currentRssi = currentRSSIData.rssiValue;
    double currentMean = currentRSSIData.mean;
    double currentStdDev = currentRSSIData.std;

    std::vector<IWiFiScanner::APInfo> matchingAPInfos;

    for (const auto &apInfo : extendedDb) {
      if (apInfo.bssid == currentBssid) {
        matchingAPInfos.push_back(apInfo);
      }
    }

    if (matchingAPInfos.empty()) {
      std::cout << "No matching entries found for BSSID: " << currentBssid
                << std::endl;
      continue;
    }

    std::cout << "Extracted entries for BSSID: " << currentBssid << std::endl;
    for (const auto &entry : matchingAPInfos) {
      std::cout << "BSSID: " << entry.bssid
                << ", Signal Strength: " << entry.signalStrength
                << ", Position: (" << entry.position[0] << ", "
                << entry.position[1] << ", " << entry.position[2] << ")"
                << std::endl;
    }

    double bestSimilarity = std::numeric_limits<double>::max();
    IWiFiScanner::APInfo bestMatchInfo;

    for (const auto &apInfo : matchingAPInfos) {
      std::vector<IWiFiScanner::APInfo> singleApVector = {apInfo};
      double similarity = calculator.calculate(
          singleApVector, {{currentBssid, currentRssi, currentMean, currentStdDev}});
      std::cout << "Calculated similarity for APInfo with BSSID: "
                << apInfo.bssid << " is " << similarity << std::endl;
      if (similarity < bestSimilarity) {
        bestSimilarity = similarity;
        bestMatchInfo = apInfo;
      }
    }

    std::cout << "Best match for BSSID: " << currentBssid << " is at position ("
              << bestMatchInfo.position[0] << ", " << bestMatchInfo.position[1]
              << ", " << bestMatchInfo.position[2] << ")"
              << " with similarity: " << bestSimilarity << std::endl;

    bestMatches.emplace_back(currentBssid, bestMatchInfo.position);

    // Mark this BSSID as processed
    processedBssids.insert(currentBssid);
  }

  return bestMatches;
}

Eigen::Vector3d FingerprintMatcher::calculateFinalPosition(
    const std::vector<std::pair<std::string, Eigen::Vector3d>> &bestMatches)
    const {
  Eigen::Vector3d finalPosition(0, 0, 0);
  double totalWeight = 0.0;

  for (const auto &match : bestMatches) {
    // Use the similarity score as the weight (the lower the score, the higher
    // the weight)
    double weight = 1.0 / (match.second.norm() +
                           1e-6); // Add a small value to avoid division by zero
    finalPosition += match.second * weight;
    totalWeight += weight;
  }

  if (totalWeight > 0) {
    finalPosition /= totalWeight;
  }

  std::cout << "Final position: (" << finalPosition[0] << ", "
            << finalPosition[1] << ", " << finalPosition[2] << ")" << std::endl;

  return finalPosition;
}
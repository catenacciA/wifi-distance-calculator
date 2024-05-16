#include "../include/FingerprintMatcher.h"
#include <cmath>
#include <limits>

FingerprintMatcher::FingerprintMatcher(const FingerprintDatabase &db)
    : database(db) {}

std::string FingerprintMatcher::match(
    const std::vector<std::pair<std::string, int>> &currentRSSIValues) const {
  const auto &db = database.getDatabase();
  std::string bestMatch;
  double bestSimilarity = std::numeric_limits<double>::max();

  for (const auto &[cellId, fingerprint] : db) {
    double similarity = calculateSimilarity(fingerprint, currentRSSIValues);
    if (similarity < bestSimilarity) {
      bestSimilarity = similarity;
      bestMatch = cellId;
    }
  }

  return bestMatch;
}

double FingerprintMatcher::calculateSimilarity(
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

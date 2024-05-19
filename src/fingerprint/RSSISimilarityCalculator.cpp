#include "../../include/fingerprint/RSSISimilarityCalculator.h"

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

#include "../include/LocationEstimator.h"

LocationEstimator::LocationEstimator(
    std::unique_ptr<WiFiScanner> scanner,
    std::unique_ptr<FingerprintMatcher> matcher)
    : scanner(std::move(scanner)), matcher(std::move(matcher)) {}

std::string LocationEstimator::estimateLocation() {
  auto apInfos = scanner->scan(true);
  std::vector<std::pair<std::string, int>> currentRSSIValues;
  for (const auto &apInfo : apInfos) {
    currentRSSIValues.emplace_back(apInfo.bssid, apInfo.signalStrength);
  }
  return matcher->match(currentRSSIValues);
}

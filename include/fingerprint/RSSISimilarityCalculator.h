#ifndef RSSISIMILARITYCALCULATOR_H
#define RSSISIMILARITYCALCULATOR_H

#include "../wifi/IWiFiScanner.h"
#include "FingerprintMatcher.h"
#include "LocationEstimator.h"
#include <cmath>
#include <map>
#include <string>
#include <utility>
#include <vector>

// Class for calculating similarity between RSSI values
class RSSISimilarityCalculator : public ISimilarityCalculator {
public:
  double calculate(const std::map<std::string, double> &fingerprint,
                   const std::vector<std::pair<std::string, int>>
                       &currentRSSIValues) const override;

  // New method to calculate similarity for extended fingerprints
  double
  calculate(const std::vector<IWiFiScanner::APInfo> &extendedFingerprint,
            const std::vector<RSSIData> &currentRSSIValues) const override;
};

#endif // RSSISIMILARITYCALCULATOR_H

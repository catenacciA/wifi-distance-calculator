#ifndef RSSISIMILARITYCALCULATOR_H
#define RSSISIMILARITYCALCULATOR_H

#include "FingerprintMatcher.h"
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
};

#endif // RSSISIMILARITYCALCULATOR_H

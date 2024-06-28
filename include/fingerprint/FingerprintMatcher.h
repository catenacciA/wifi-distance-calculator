// FingerprintMatcher.h

#ifndef FINGERPRINTMATCHER_H
#define FINGERPRINTMATCHER_H

#include "FingerprintDatabase.h"
#include "RSSIData.h"
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

// Interface for a similarity calculator
class ISimilarityCalculator {
public:
  virtual ~ISimilarityCalculator() = default;
  virtual double calculate(const std::map<std::string, double> &fingerprint,
                           const std::vector<std::pair<std::string, int>>
                               &currentRSSIValues) const = 0;
  virtual double
  calculate(const std::vector<IWiFiScanner::APInfo> &extendedFingerprint,
            const std::vector<RSSIData> &currentRSSIValues) const = 0;
};

// Class for matching current RSSI values to a fingerprint database
class FingerprintMatcher {
public:
  FingerprintMatcher(const FingerprintDatabase &database,
                     ISimilarityCalculator &calculator);
  std::string match(
      const std::vector<std::pair<std::string, int>> &currentRSSIValues) const;

  // New methods to match against the extended and AP databases
  std::vector<std::pair<std::string, Eigen::Vector3d>>
  matchExtended(const std::vector<RSSIData> &currentRSSIValues) const;

  Eigen::Vector3d calculateFinalPosition(
      const std::vector<std::pair<std::string, Eigen::Vector3d>> &bestMatches)
      const;

private:
  const FingerprintDatabase &database;
  ISimilarityCalculator &calculator;
};

#endif // FINGERPRINTMATCHER_H

#ifndef FINGERPRINTMATCHER_H
#define FINGERPRINTMATCHER_H

#include "FingerprintDatabase.h"
#include <vector>
#include <string>
#include <utility>
#include <limits>

// Interface for a similarity calculator
class ISimilarityCalculator {
public:
    virtual ~ISimilarityCalculator() = default;
    virtual double calculate(const std::map<std::string, double>& fingerprint, const std::vector<std::pair<std::string, int>>& currentRSSIValues) const = 0;
};

// Class for matching current RSSI values to a fingerprint database
class FingerprintMatcher {
public:
    FingerprintMatcher(const FingerprintDatabase& database, ISimilarityCalculator& calculator);
    std::string match(const std::vector<std::pair<std::string, int>>& currentRSSIValues) const;

private:
    const FingerprintDatabase& database;
    ISimilarityCalculator& calculator;
};

#endif // FINGERPRINTMATCHER_H

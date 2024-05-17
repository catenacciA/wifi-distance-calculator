#ifndef FINGERPRINTMATCHER_H
#define FINGERPRINTMATCHER_H

#include "FingerprintDatabase.h"
#include <vector>
#include <string>
#include <utility>

// Class for matching current RSSI values to a fingerprint database
class FingerprintMatcher {
public:
    FingerprintMatcher(const FingerprintDatabase& database);
    std::string match(const std::vector<std::pair<std::string, int>>& currentRSSIValues) const;

private:
    const FingerprintDatabase& database;
    double calculateSimilarity(const std::map<std::string, double>& fingerprint, const std::vector<std::pair<std::string, int>>& currentRSSIValues) const;
};

#endif // FINGERPRINTMATCHER_H

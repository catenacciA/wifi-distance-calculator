// LocationEstimator.h

#ifndef LOCATIONESTIMATOR_H
#define LOCATIONESTIMATOR_H

#include "../wifi/WiFiScanner.h"
#include "FingerprintMatcher.h"
#include "RSSIData.h"
#include <memory>
#include <string>
#include <vector>

// Class for estimating the current location of a device
class LocationEstimator {
public:
  // Constructor with WiFiScanner
  LocationEstimator(std::unique_ptr<WiFiScanner> scanner,
                    std::unique_ptr<FingerprintMatcher> matcher);

  // Constructor without WiFiScanner
  LocationEstimator(std::unique_ptr<FingerprintMatcher> matcher);

  std::string estimateLocation();
  std::vector<std::pair<std::string, Eigen::Vector3d>>
  estimateExtendedLocation(const std::string &rssiFile);
  std::vector<RSSIData> readRSSIValuesFromFile(const std::string &filename);

private:
  std::unique_ptr<WiFiScanner> scanner;
  std::unique_ptr<FingerprintMatcher> matcher;
};

#endif // LOCATIONESTIMATOR_H

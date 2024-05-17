#ifndef LOCATIONESTIMATOR_H
#define LOCATIONESTIMATOR_H

#include "../wifi/WiFiScanner.h"
#include "../fingerprint/FingerprintMatcher.h"

// Class for estimating the current location of a device
class LocationEstimator {
public:
    LocationEstimator(std::unique_ptr<WiFiScanner> scanner, std::unique_ptr<FingerprintMatcher> matcher);
    std::string estimateLocation();

private:
    std::unique_ptr<WiFiScanner> scanner;
    std::unique_ptr<FingerprintMatcher> matcher;
};

#endif // LOCATIONESTIMATOR_H

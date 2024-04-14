// DistanceCalculator.cpp
#include "include/DistanceCalculator.hpp"
#include "include/WiFiScanner.hpp"

#include <cmath>
#include <string>

// Implementing the static member of DistanceCalculator
double DistanceCalculator::calculateDistance(double rssi, double referenceRssi,
                                             double pathLossExponent,
                                             double referenceDistance) {
  return referenceDistance *
         std::pow(10.0, (rssi - referenceRssi) / (10.0 * pathLossExponent));
}

// Example function that uses AccessPointInfo
double DistanceCalculator::calculateDistanceToAP(
    const WiFiScanner::AccessPointInfo &ap) {
  // Sample reference values, these should be based on actual measurements
  double referenceRssi = -39.00; // The RSSI value at 1 meters
  double pathLossExponent = 4.0; // Typical indoor path loss exponent
  double referenceDistance = 1;  // Reference distance in meters

  return DistanceCalculator::calculateDistance(
      ap.signalStrengthDbm, referenceRssi, pathLossExponent, referenceDistance);
}

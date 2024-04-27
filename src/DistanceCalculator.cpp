// DistanceCalculator.cpp
#include "include/DistanceCalculator.hpp"
#include "include/WiFiScanner.hpp"

#include <cmath>
#include <string>

/**
 * @brief Calculates the distance to the WiFi access point based on the Received
 * Signal Strength Indicator (RSSI).
 *
 * @param rssi The current RSSI value in decibels (dBm).
 * @param referenceRssi The RSSI value at a known reference distance, typically
 * in dBm.
 * @param pathLossExponent The path loss exponent, which varies depending on the
 * environmental conditions. Common values are 2.0 to 4.0, where 2.0 might
 * represent free space, and higher values represent environments with more
 * obstacles.
 * @param referenceDistance The known reference distance from the transmitter in
 * meters (e.g., 1 meter).
 * @return double The estimated distance to the access point in meters.
 */
double DistanceCalculator::calculateDistance(double rssi, double referenceRssi,
                                             double pathLossExponent,
                                             double referenceDistance) {
  return referenceDistance *
         std::pow(10.0, (referenceRssi - rssi) / (10.0 * pathLossExponent));
}

/**
 * @brief Calculates the distance to a specific WiFi access point using
 * predefined reference values.
 *
 * This function utilizes the calculateDistance function by providing it with
 * the RSSI values and reference parameters specific to a typical indoor
 * environment.
 *
 * @param ap The AccessPointInfo structure containing details about the WiFi
 * access point, including its signal strength.
 * @return double The calculated distance to the access point in meters.
 */
double DistanceCalculator::calculateDistanceToAP(
    const WiFiScanner::AccessPointInfo &ap) {
  // Sample reference values, these should be based on actual measurements
  double referenceRssi = 41.72; // The RSSI value at 1 meter
  double pathLossExponent = 14.61; // Typical indoor path loss exponent
  double referenceDistance = 1;  // Reference distance in meters

  return DistanceCalculator::calculateDistance(
      ap.signalStrengthDbm, referenceRssi, pathLossExponent, referenceDistance);
}

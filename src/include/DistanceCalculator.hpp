#ifndef DISTANCE_CALCULATOR_H
#define DISTANCE_CALCULATOR_H

#include "WiFiScanner.hpp"

class DistanceCalculator {
public:
  /**
   * Calculates the estimated distance from a transmitter based on the RSSI
   * value.
   *
   * @param rssi The Received Signal Strength Indicator in dBm.
   * @param referenceRssi The received signal strength at the reference distance
   * in dBm.
   * @param pathLossExponent The path loss exponent, which depends on the
   * propagation environment.
   * @param referenceDistance The reference distance from the transmitter in
   * meters where referenceRssi is measured.
   * @return The estimated distance to the transmitter in meters.
   *
   * Note: Implementation of this function will use the path loss model formula:
   *   distance = referenceDistance * 10 ^ ((rssi - referenceRssi) / (10 *
   * pathLossExponent))
   */
  static double calculateDistance(double rssi, double referenceRssi,
                                  double pathLossExponent,
                                  double referenceDistance);

  /**
   * Calculates the estimated distance to an Access Point based on the RSSIs
   *
   * @param ap
   * @return double
   */
  static double calculateDistanceToAP(const WiFiScanner::AccessPointInfo &ap);
};

#endif // DISTANCE_CALCULATOR_H

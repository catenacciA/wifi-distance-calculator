#include "../../include/distance/LogDistanceCalculator.h"
#include <cmath>

LogDistanceCalculator::LogDistanceCalculator(double pathLossExponent,
                                             double referenceDistance,
                                             double referenceRSSI, double sigma)
    : pathLossExponent(pathLossExponent), referenceDistance(referenceDistance),
      referenceRSSI(referenceRSSI), sigma(sigma), gen(std::random_device{}()),
      dist(0, sigma) {}

double LogDistanceCalculator::calculateDistance(int signalStrength) const {
  double x_sigma = dist(gen);
  double adjustedRSSI = signalStrength + x_sigma;
  return referenceDistance * std::pow(10.0, (referenceRSSI - adjustedRSSI) /
                                                (10.0 * pathLossExponent));
}

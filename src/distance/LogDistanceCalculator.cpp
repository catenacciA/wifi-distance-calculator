#include "../../include/distance/LogDistanceCalculator.h"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

LogDistanceCalculator::LogDistanceCalculator(double pathLossExponent,
                                             double referenceDistance,
                                             double referencePathLoss,
                                             double transmitPower, double sigma)
    : pathLossExponent(pathLossExponent), referenceDistance(referenceDistance),
      referencePathLoss(referencePathLoss), transmitPower(transmitPower),
      sigma(sigma), gen(std::random_device{}()), dist(0, sigma) {}

double LogDistanceCalculator::movingAverage(
    const std::vector<int> &signalStrengths) const {
  int windowSize = 5; // You can adjust the window size as needed
  std::vector<double> smoothedSignal;
  for (size_t i = 0; i < signalStrengths.size(); ++i) {
    int start = std::max(0, static_cast<int>(i) - windowSize + 1);
    int end = i + 1;
    double sum = std::accumulate(signalStrengths.begin() + start,
                                 signalStrengths.begin() + end, 0.0);
    smoothedSignal.push_back(sum / (end - start));
  }
  return smoothedSignal.back();
}

double LogDistanceCalculator::calculateDistance(int signalStrength) const {
  double pathLoss =
      transmitPower -
      signalStrength;
  double x_sigma = dist(gen);
  double adjustedPathLoss =
      pathLoss - x_sigma;
  return referenceDistance *
         std::pow(10.0, (adjustedPathLoss - referencePathLoss) /
                            (10.0 * pathLossExponent));
}

double LogDistanceCalculator::calculateDistance(
    const std::vector<int> &signalStrengths) const {
  double smoothedSignal = movingAverage(signalStrengths);
  double pathLoss =
      transmitPower - smoothedSignal;
  double x_sigma = dist(gen);
  double adjustedPathLoss =
      pathLoss - x_sigma;
  return referenceDistance *
         std::pow(10.0, (adjustedPathLoss - referencePathLoss) /
                            (10.0 * pathLossExponent));
}
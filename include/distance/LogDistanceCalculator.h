#ifndef LOGDISTANCECALCULATOR_H
#define LOGDISTANCECALCULATOR_H

#include "IDistanceCalculator.h"
#include <random>
#include <vector>

// Class for calculating distance based on the log-distance path loss model
class LogDistanceCalculator : public IDistanceCalculator {
public:
  LogDistanceCalculator(double pathLossExponent, double referenceDistance,
                        double referencePathLoss, double transmitPower,
                        double sigma);
  double calculateDistance(int signalStrength) const override;
  double calculateDistance(const std::vector<int> &signalStrengths) const;

  // Getter for model parameters
  std::tuple<double, double, double, double, double>
  getModelParameters() const {
    return std::make_tuple(pathLossExponent, referenceDistance,
                           referencePathLoss, transmitPower, sigma);
  }

private:
  double pathLossExponent;
  double referenceDistance;
  double referencePathLoss;
  double transmitPower;
  double sigma;

  mutable std::mt19937 gen;
  mutable std::normal_distribution<double> dist;

  double movingAverage(const std::vector<int> &signalStrengths) const;
};

#endif // LOGDISTANCECALCULATOR_H

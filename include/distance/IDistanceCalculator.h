#ifndef DISTANCECALCULATOR_H
#define DISTANCECALCULATOR_H

#include <tuple>
#include <vector>

// The DistanceCalculator class is an abstract class that defines the interface
// for all distance calculators.
class IDistanceCalculator {
public:
  virtual ~IDistanceCalculator() = default;
  virtual double calculateDistance(int signalStrength) const = 0;

  // New method to retrieve model parameters
  virtual std::tuple<double, double, double, double, double>
  getModelParameters() const = 0;

  // New method to calculate distance from a vector of signal strengths
  virtual double
  calculateDistance(const std::vector<int> &signalStrengths) const = 0;
};

#endif // DISTANCECALCULATOR_H

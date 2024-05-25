#include "../../include/utility/MathUtils.h"

double MathUtils::calculateMean(const std::vector<double> &values) {
  return std::accumulate(values.begin(), values.end(), 0.0) / values.size();
}

double MathUtils::calculateStandardDeviation(const std::vector<double> &values,
                                             double mean) {
  if (values.size() <= 1) {
    throw std::invalid_argument(
        "Not enough data to calculate standard deviation");
  }

  double variance = std::accumulate(values.begin(), values.end(), 0.0,
                                    [mean](double accum, double value) {
                                      double diff = value - mean;
                                      return accum + diff * diff;
                                    }) /
                    (values.size() - 1);

  return std::sqrt(variance);
}

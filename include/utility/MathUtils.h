#ifndef MATHUTILS_H
#define MATHUTILS_H

#include <cmath>
#include <numeric>
#include <stdexcept>
#include <vector>

class MathUtils {
public:
  static double calculateMean(const std::vector<double> &values);
  static double calculateStandardDeviation(const std::vector<double> &values,
                                           double mean);
};

#endif // MATHUTILS_H

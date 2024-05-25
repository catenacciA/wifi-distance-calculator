#ifndef IPOSITION_ESTIMATOR_H
#define IPOSITION_ESTIMATOR_H

#include "../wifi/WiFiScanner.h"
#include <Eigen/Dense>

class IPositionEstimator {
public:
  virtual ~IPositionEstimator() = default;
  virtual Eigen::Vector3d estimatePosition(WiFiScanner &scanner) = 0;
};

#endif // IPOSITION_ESTIMATOR_H

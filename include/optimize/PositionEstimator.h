#ifndef POSITION_ESTIMATOR_H
#define POSITION_ESTIMATOR_H

#include "../wifi/WiFiScanner.h"
#include "IPositionEstimator.h"
#include <Eigen/Dense>
#include <map>
#include <memory>
#include <vector>

class PositionEstimator : public IPositionEstimator {
public:
    PositionEstimator();

    Eigen::Vector3d estimatePosition(WiFiScanner &scanner) override;

private:
    Eigen::Vector3d calculateInitialGuess(const std::vector<WiFiScanner::APInfo> &meanAPInfos);
};

#endif // POSITION_ESTIMATOR_H

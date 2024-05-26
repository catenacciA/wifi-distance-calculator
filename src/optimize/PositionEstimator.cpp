#include "../../include/optimize/PositionEstimator.h"
#include "../../include/distance/LogDistanceCalculator.h"
#include "../../include/optimize/DistanceFunctor.h"
#include "../../include/utility/MathUtils.h"
#include "../../include/utility/ProgressBar.h"
#include <algorithm>
#include <cmath>
#include <gtsam/linear/KalmanFilter.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/expressions.h>
#include <iostream>
#include <map>
#include <numeric>
#include <vector>

using namespace gtsam::symbol_shorthand;

PositionEstimator::PositionEstimator() {}

Eigen::Vector3d PositionEstimator::calculateInitialGuess(
    const std::vector<WiFiScanner::APInfo> &meanAPInfos) {
  std::cout << "Calculating initial guess..." << std::endl;
  Eigen::Vector3d sumPosition(0.0, 0.0, 0.0);
  int validAPCount = 0;

  for (const auto &apInfo : meanAPInfos) {
    std::cout << "AP Info position: " << apInfo.position.transpose()
              << std::endl;
    if (apInfo.position.x() >= 0 && apInfo.position.y() >= 0 &&
        apInfo.position.z() >= 0) {
      sumPosition += apInfo.position;
      ++validAPCount;
    }
  }

  if (validAPCount > 0) {
    Eigen::Vector3d initialGuess = sumPosition / validAPCount;
    std::cout << "Initial guess calculated: " << initialGuess.transpose()
              << std::endl;
    return initialGuess;
  } else {
    std::cout << "No valid AP positions found, defaulting to origin."
              << std::endl;
    return Eigen::Vector3d(0.0, 0.0,
                           0.0); // Default to origin if no valid AP positions
  }
}

Eigen::Vector3d PositionEstimator::estimatePosition(WiFiScanner &scanner) {
  std::cout << "Estimating position..." << std::endl;
  std::map<std::string, std::vector<double>> signalStrengthMeasurements;
  ProgressBar progressBar;
  const int totalScans = 10;

  for (int i = 0; i < totalScans; ++i) {
    auto apInfos = scanner.scan(true);
    for (const auto &apInfo : apInfos) {
      signalStrengthMeasurements[apInfo.bssid].push_back(apInfo.signalStrength);
    }
    progressBar.display(i + 1, totalScans);
  }

  std::cout << std::endl;
  std::vector<WiFiScanner::APInfo> meanAPInfos;
  std::map<std::string, Eigen::MatrixXd> covarianceMatrices;

  int apCount = signalStrengthMeasurements.size();
  std::map<std::string, int> bssidToIndex;
  int index = 0;

  for (const auto &entry : signalStrengthMeasurements) {
    const std::string &bssid = entry.first;
    const std::vector<double> &signalStrengths = entry.second;

    if (signalStrengths.size() <= 1) {
      continue; // Skip if there are not enough measurements to construct a
                // covariance matrix
    }

    double meanSignalStrength = MathUtils::calculateMean(signalStrengths);
    double stdDev = MathUtils::calculateStandardDeviation(signalStrengths,
                                                          meanSignalStrength);

    IDistanceCalculator *distanceCalculator =
        scanner.getDistanceCalculator(bssid);
    if (!distanceCalculator) {
      continue;
    }

    double meanDistance =
        distanceCalculator->calculateDistance(meanSignalStrength);

    WiFiScanner::APInfo meanAPInfo;
    meanAPInfo.bssid = bssid;
    meanAPInfo.distance = meanDistance;
    meanAPInfo.position = scanner.getAPPositions().at(bssid);

    meanAPInfos.push_back(meanAPInfo);
    bssidToIndex[bssid] = index++;

    // Construct the scalar variance for this AP
    double variance =
        stdDev * stdDev + 1e-2; // Further increased regularization term
    covarianceMatrices[bssid] = Eigen::MatrixXd::Constant(1, 1, variance);

    std::cout << "AP Info: BSSID=" << bssid
              << ", Position=" << meanAPInfo.position.transpose()
              << ", Distance=" << meanAPInfo.distance << std::endl;
  }

  std::cout << "Covariance Matrices: " << std::endl;
  for (const auto &entry : covarianceMatrices) {
    std::cout << "BSSID=" << entry.first << "\n" << entry.second << std::endl;
  }

  gtsam::NonlinearFactorGraph graph;
  gtsam::Key vehiclePositionKey = X(0);

  for (size_t i = 0; i < meanAPInfos.size(); ++i) {
    const auto &apInfo = meanAPInfos[i];
    Eigen::Vector3d apPosition = apInfo.position;
    double observedDistance = apInfo.distance;
    Eigen::MatrixXd covarianceMatrix = covarianceMatrices[apInfo.bssid];

    // Ensure the covariance matrix is the correct size
    if (covarianceMatrix.rows() != 1 || covarianceMatrix.cols() != 1) {
      std::cerr << "Covariance matrix for " << apInfo.bssid
                << " has incorrect dimensions." << std::endl;
      continue;
    }

    auto noiseModel = gtsam::noiseModel::Gaussian::Covariance(covarianceMatrix);

    gtsam::Point3 apPos(apPosition.x(), apPosition.y(), apPosition.z());
    gtsam::Expression<gtsam::Point3> apPositionExpr(apPos);

    gtsam::Expression<double> distanceExpression(
        DistanceFunctor(), gtsam::Expression<gtsam::Point3>(vehiclePositionKey),
        apPositionExpr);

    auto distanceFactor = gtsam::ExpressionFactor<double>(
        noiseModel, observedDistance, distanceExpression);

    graph.add(distanceFactor);
    std::cout << "Added distance factor for AP: " << apInfo.bssid << std::endl;
  }

  Eigen::Vector3d initialGuess = calculateInitialGuess(meanAPInfos);
  initialGuess +=
      Eigen::Vector3d(0.1, 0.1, 0.1); // Slight adjustment to initial guess
  std::cout << "Initial Guess: " << initialGuess.transpose() << std::endl;

  // Kalman Filter Initialization
  gtsam::Vector x0 =
      (gtsam::Vector(3) << initialGuess.x(), initialGuess.y(), initialGuess.z())
          .finished();
  gtsam::Matrix P0 =
      gtsam::Matrix::Identity(3, 3); // Initial covariance estimate

  gtsam::KalmanFilter kalmanFilter(x0.size());
  auto state = kalmanFilter.init(x0, P0);

  for (const auto &apInfo : meanAPInfos) {
    Eigen::Vector3d apPosition = apInfo.position;
    double observedDistance = apInfo.distance;

    // Measurement model (position to distance)
    gtsam::Matrix H(1, 3); // 1x3 Jacobian matrix
    Eigen::Vector3d diff = x0.head<3>() - apPosition;
    double norm = diff.norm();
    H << diff.transpose() / norm;

    gtsam::Vector z(1);
    z << observedDistance;

    auto noiseModel = gtsam::noiseModel::Diagonal::Variances(
        covarianceMatrices[apInfo.bssid].col(0));

    // Kalman Filter Update
    state = kalmanFilter.update(state, H, z, noiseModel);
  }

  gtsam::Vector initialGuessFiltered = state->mean();
  std::cout << "Kalman Filtered Initial Guess: "
            << initialGuessFiltered.transpose() << std::endl;

  gtsam::LevenbergMarquardtParams params;
  params.setMaxIterations(100);
  params.setRelativeErrorTol(1e-5);
  params.setAbsoluteErrorTol(1e-5);

  gtsam::Values initialEstimate;
  initialEstimate.insert(vehiclePositionKey,
                         gtsam::Point3(initialGuessFiltered(0),
                                       initialGuessFiltered(1),
                                       initialGuessFiltered(2)));

  try {
    std::cout << "Starting optimization..." << std::endl;
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initialEstimate,
                                                 params);
    gtsam::Values result = optimizer.optimize();
    std::cout << "Optimization finished." << std::endl;

    gtsam::Point3 estimatedPosition =
        result.at<gtsam::Point3>(vehiclePositionKey);
    Eigen::Vector3d estimatedPosVec(
        estimatedPosition.x(), estimatedPosition.y(), estimatedPosition.z());

    // Ensure positive values higher than 0
    estimatedPosVec = estimatedPosVec.cwiseMax(1e-6);

    return estimatedPosVec;
  } catch (const std::exception &e) {
    std::cerr << "Optimization failed: " << e.what() << std::endl;
    throw;
  }
}

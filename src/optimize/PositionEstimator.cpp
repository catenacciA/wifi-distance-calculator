#include "../../include/optimize/PositionEstimator.h"
#include "../../include/distance/LogDistanceCalculator.h"
#include "../../include/optimize/Distance3DFunctor.h"
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

// Calculate the initial guess for the position based on the mean AP positions
// Mathematically, the initial guess is the weighted average of the AP positions
// weighted by the inverse of the distance
Eigen::Vector3d PositionEstimator::calculateInitialGuess(
    const std::vector<WiFiScanner::APInfo> &meanAPInfos) {
  std::cout << "Calculating initial guess..." << std::endl;
  Eigen::Vector3d sumPosition(0.0, 0.0, 0.0);
  double totalWeight = 0.0;

  for (const auto &apInfo : meanAPInfos) {
    if (apInfo.position.x() >= 0 && apInfo.position.y() >= 0 &&
        apInfo.position.z() >= 0) {
      double weight = 1.0 / (apInfo.distance + 1e-6); // Weighted by distance
      sumPosition += weight * apInfo.position;
      totalWeight += weight;
    }
  }

  if (totalWeight > 0) {
    Eigen::Vector3d initialGuess = sumPosition / totalWeight;
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
  std::map<std::string, Eigen::Matrix3d> covarianceMatrices;

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

    // Construct the 3x3 covariance matrix for this AP
    Eigen::Matrix3d covarianceMatrix = Eigen::Matrix3d::Identity();
    covarianceMatrix.diagonal() << stdDev * stdDev, stdDev * stdDev,
        stdDev * stdDev;
    covarianceMatrices[bssid] = covarianceMatrix;

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
    Eigen::Matrix3d covarianceMatrix = covarianceMatrices[apInfo.bssid];

    auto noiseModel = gtsam::noiseModel::Gaussian::Covariance(covarianceMatrix);

    gtsam::Point3 apPos(apPosition.x(), apPosition.y(), apPosition.z());
    gtsam::Expression<gtsam::Point3> apPositionExpr(apPos);

    gtsam::Expression<gtsam::Point3> positionExpression{
        gtsam::Expression<gtsam::Point3>(vehiclePositionKey)};

    // Create the difference functor and factor
    Distance3DFunctor distance3DFunctor;
    auto differenceExpression = gtsam::Expression<gtsam::Vector3>(
        std::function<gtsam::Vector3(
            const gtsam::Point3 &, const gtsam::Point3 &,
            gtsam::OptionalJacobian<3, 3>, gtsam::OptionalJacobian<3, 3>)>(
            distance3DFunctor),
        positionExpression, apPositionExpr);

    auto positionFactor = gtsam::ExpressionFactor<gtsam::Vector3>(
        noiseModel, Eigen::Vector3d::Zero(), differenceExpression);

    graph.add(positionFactor);
    std::cout << "Added position factor for AP: " << apInfo.bssid << std::endl;
  }

  // Calculate initial guess
  Eigen::Vector3d initialGuess = calculateInitialGuess(meanAPInfos);
  std::cout << "Initial Guess: " << initialGuess.transpose() << std::endl;

  // Nonlinear Optimization
  gtsam::LevenbergMarquardtParams params;
  params.setMaxIterations(100);
  params.setRelativeErrorTol(1e-5);
  params.setAbsoluteErrorTol(1e-5);

  gtsam::Values initialEstimate;
  initialEstimate.insert(
      vehiclePositionKey,
      gtsam::Point3(initialGuess(0), initialGuess(1), initialGuess(2)));

  // Initialize Kalman Filter
  const size_t state_dim = 3;
  gtsam::KalmanFilter kf(state_dim);
  gtsam::Vector initialMean = initialGuess;
  gtsam::Matrix initialCovariance =
      Eigen::MatrixXd::Identity(state_dim, state_dim);
  gtsam::KalmanFilter::State kfState = kf.init(initialMean, initialCovariance);

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

    // Sanity check to ensure estimated position is reasonable
    if (estimatedPosVec.hasNaN()) {
      std::cerr << "Estimated position is invalid: "
                << estimatedPosVec.transpose() << std::endl;
      throw std::runtime_error("Invalid estimated position.");
    }

    std::cout << "Estimated Position (before Kalman Filter): "
              << estimatedPosVec.transpose() << std::endl;

    // Kalman Filter Update
    gtsam::Vector measurement = estimatedPosVec;
    gtsam::Matrix measurementCovariance =
        Eigen::MatrixXd::Identity(state_dim, state_dim);
    gtsam::Matrix H =
        Eigen::MatrixXd::Identity(state_dim, state_dim); // Measurement matrix

    gtsam::SharedDiagonal noise =
        gtsam::noiseModel::Diagonal::Sigmas(measurementCovariance.diagonal());
    kfState = kf.update(kfState, H, measurement, noise);

    // Extract the mean from the Kalman Filter state
    gtsam::Vector filteredMean = kfState->mean();
    std::cout << "Estimated Position (after Kalman Filter): "
              << filteredMean.transpose() << std::endl;
    return filteredMean;
  } catch (const std::exception &e) {
    std::cerr << "Optimization failed: " << e.what() << std::endl;
    throw;
  }
}

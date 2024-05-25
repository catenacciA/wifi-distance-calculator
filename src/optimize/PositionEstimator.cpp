#include "../../include/optimize/PositionEstimator.h"
#include "../../include/distance/LogDistanceCalculator.h"
#include "../../include/optimize/DistanceFunctor.h"
#include "../../include/utility/MathUtils.h"
#include "../../include/utility/ProgressBar.h"
#include <algorithm>
#include <cmath>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/expressions.h>
#include <iostream>
#include <map>
#include <numeric>
#include <vector>

using namespace gtsam::symbol_shorthand;

PositionEstimator::PositionEstimator() {}

Eigen::Vector3d PositionEstimator::calculateInitialGuess(
    const std::vector<WiFiScanner::APInfo> &meanAPInfos) {
  Eigen::Vector3d meanPosition(0, 0, 0);
  for (const auto &apInfo : meanAPInfos) {
    meanPosition += apInfo.position;
  }
  if (!meanAPInfos.empty()) {
    meanPosition /= meanAPInfos.size();
  }
  return meanPosition;
}

Eigen::Vector3d PositionEstimator::estimatePosition(WiFiScanner &scanner) {
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
  std::map<std::string, double> stdDevs;

  for (const auto &entry : signalStrengthMeasurements) {
    const std::string &bssid = entry.first;
    const std::vector<double> &signalStrengths = entry.second;

    if (signalStrengths.empty()) {
      continue;
    }

    double meanSignalStrength = MathUtils::calculateMean(signalStrengths);
    double stdDev = MathUtils::calculateStandardDeviation(signalStrengths,
                                                          meanSignalStrength);
    stdDevs[bssid] = stdDev;

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
  }

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initialEstimate;
  gtsam::Key vehiclePositionKey = X(0);

  Eigen::Vector3d initialGuess = calculateInitialGuess(meanAPInfos);
  initialEstimate.insert(
      vehiclePositionKey,
      gtsam::Point3(initialGuess.x(), initialGuess.y(), initialGuess.z()));

  auto priorNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1.0);
  double z_prior_mean = 1.0;
  double z_prior_noise_sigma = 0.5;
  graph.add(gtsam::PriorFactor<gtsam::Point3>(
      vehiclePositionKey,
      gtsam::Point3(initialGuess.x(), initialGuess.y(), z_prior_mean),
      gtsam::noiseModel::Isotropic::Sigma(3, z_prior_noise_sigma)));

  for (const auto &apInfo : meanAPInfos) {
    Eigen::Vector3d apPosition = apInfo.position;
    double observedDistance = apInfo.distance;

    double noiseSigma = std::max(stdDevs[apInfo.bssid], 1.0);
    auto noiseModel = gtsam::noiseModel::Isotropic::Sigma(1, noiseSigma);

    gtsam::Point3 apPos(apPosition.x(), apPosition.y(), apPosition.z());
    gtsam::Expression<gtsam::Point3> apPositionExpr(apPos);

    gtsam::Expression<double> distanceExpression(
        DistanceFunctor(), gtsam::Expression<gtsam::Point3>(vehiclePositionKey),
        apPositionExpr);

    auto distanceFactor = gtsam::ExpressionFactor<double>(
        noiseModel, observedDistance, distanceExpression);

    graph.add(distanceFactor);
  }

  gtsam::LevenbergMarquardtParams params;
  params.setMaxIterations(100);
  params.setRelativeErrorTol(1e-5);
  params.setAbsoluteErrorTol(1e-5);
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, initialEstimate, params);
  gtsam::Values result = optimizer.optimize();

  gtsam::Point3 estimatedPosition =
      result.at<gtsam::Point3>(vehiclePositionKey);

  return Eigen::Vector3d(estimatedPosition.x(), estimatedPosition.y(),
                         estimatedPosition.z());
}

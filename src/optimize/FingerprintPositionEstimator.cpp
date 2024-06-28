#include <Eigen/Dense>
#include <fstream>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/sam/RangeFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "../../include/optimize/Distance3DFunctor.h"
#include "../../include/optimize/FingerprintPositionEstimator.h"

using namespace gtsam::symbol_shorthand;

FingerprintPositionEstimator::FingerprintPositionEstimator(
    const std::string &fingerprintFile, const std::string &apFile)
    : _csvParser(CSVDatabaseParser()),
      _similarityCalculator(RSSISimilarityCalculator()),
      _fingerprintDB(
          _csvParser.parseExtendedFingerprintDatabase(fingerprintFile)),
      _apDatabase(_csvParser.parseAPDatabase(apFile)),
      _fingerprintDatabase(_fingerprintDB),
      _matcher(
          FingerprintMatcher(_fingerprintDatabase, _similarityCalculator)) {}

std::vector<gtsam::Pose3>
FingerprintPositionEstimator::parseLIOSLAMFile(const std::string &fileName) {
  std::ifstream file(fileName);
  if (!file.is_open()) {
    throw std::runtime_error("Could not open LIO SLAM file.");
  }

  std::vector<gtsam::Pose3> poses;
  std::string line;
  while (std::getline(file, line)) {
    std::istringstream iss(line);
    double timestamp, tx, ty, tz, qx, qy, qz, qw;
    if (iss >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw) {
      gtsam::Point3 position(tx, ty, tz);
      gtsam::Rot3 rotation(gtsam::Quaternion(qw, qx, qy, qz));
      poses.emplace_back(rotation, position);
    }
  }
  return poses;
}

Eigen::Vector3d FingerprintPositionEstimator::estimatePosition(
    const std::vector<RSSIData> &currentRSSIValues) {
  auto bestMatches = _matcher.matchExtended(currentRSSIValues);
  if (bestMatches.empty()) {
    throw std::runtime_error(
        "No matching APs found in the fingerprint database.");
  }

  Eigen::Vector3d initialEstimate =
      _matcher.calculateFinalPosition(bestMatches);

  auto [positions, distances, covarianceMatrices] =
      prepareDataForOptimization(bestMatches, initialEstimate);

  gtsam::NonlinearFactorGraph graph;
  gtsam::Key vehiclePositionKey = X(0);

  addPriorFactors(graph, vehiclePositionKey, initialEstimate);
  addRangeFactors(graph, vehiclePositionKey, positions, distances,
                  covarianceMatrices);

  auto trajectory = parseLIOSLAMFile("../lio_slam.txt");
  addPosePriors(graph, trajectory);

  gtsam::Values initialEstimateValues = initializeEstimateValues(
      vehiclePositionKey, initialEstimate, positions, trajectory);

  std::cout << "Graph: " << std::endl;
  graph.print();

  return optimize(graph, initialEstimateValues, vehiclePositionKey);
}

std::tuple<std::vector<Eigen::Vector3d>, std::vector<double>,
           std::vector<Eigen::Matrix3d>>
FingerprintPositionEstimator::prepareDataForOptimization(
    const std::vector<std::pair<std::string, Eigen::Vector3d>> &bestMatches,
    const Eigen::Vector3d &initialEstimate) {

  std::vector<Eigen::Vector3d> positions;
  std::vector<double> distances;
  std::vector<Eigen::Matrix3d> covarianceMatrices;

  for (const auto &[bssid, _] : bestMatches) {
    auto it = _apDatabase.find(bssid);
    if (it != _apDatabase.end()) {
      const Eigen::Vector3d &knownAPPosition = it->second.first;
      const Eigen::Matrix3d &covarianceMatrix = it->second.second;
      double distance = (knownAPPosition - initialEstimate).norm();
      positions.push_back(knownAPPosition);
      distances.push_back(distance);
      covarianceMatrices.push_back(covarianceMatrix);

      std::cout << "MAC Address: " << bssid
                << ", Known AP Position: " << knownAPPosition.transpose()
                << ", Initial Estimate: " << initialEstimate.transpose()
                << ", Distance: " << distance << std::endl;
    } else {
      std::cerr << "MAC Address: " << bssid << " not found in AP database."
                << std::endl;
      throw std::runtime_error("MAC Address " + bssid +
                               " not found in AP database.");
    }
  }

  return {positions, distances, covarianceMatrices};
}

void FingerprintPositionEstimator::addPriorFactors(
    gtsam::NonlinearFactorGraph &graph, gtsam::Key vehiclePositionKey,
    const Eigen::Vector3d &initialEstimate) {
  auto priorNoise = gtsam::noiseModel::Diagonal::Sigmas(
      (Eigen::Vector3d() << 1.0, 1.0, 1.0).finished());
  graph.add(gtsam::PriorFactor<gtsam::Point3>(
      vehiclePositionKey, gtsam::Point3(initialEstimate), priorNoise));
}

void FingerprintPositionEstimator::addRangeFactors(
    gtsam::NonlinearFactorGraph &graph, gtsam::Key vehiclePositionKey,
    const std::vector<Eigen::Vector3d> &positions,
    const std::vector<double> &distances,
    const std::vector<Eigen::Matrix3d> &covarianceMatrices) {
  // Assuming a standard deviation of 1 meter for range measurements
  // TODO: Use actual range measurements
  auto rangeNoiseModel = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);

  for (size_t i = 0; i < positions.size(); ++i) {
    gtsam::Key apKey = gtsam::Symbol('A', i);
    // A is the key for the AP position
    auto apPriorNoise =
        gtsam::noiseModel::Gaussian::Covariance(covarianceMatrices[i]);
    graph.add(gtsam::PriorFactor<gtsam::Point3>(
        apKey,
        gtsam::Point3(positions[i].x(), positions[i].y(), positions[i].z()),
        apPriorNoise));
    graph.add(gtsam::RangeFactor<gtsam::Point3, gtsam::Point3>(
        vehiclePositionKey, apKey, distances[i], rangeNoiseModel));
  }
}

// Function to add pose priors
void FingerprintPositionEstimator::addPosePriors(
    gtsam::NonlinearFactorGraph &graph,
    const std::vector<gtsam::Pose3> &trajectory) {
  for (size_t i = 0; i < trajectory.size(); ++i) {
    gtsam::Key poseKey = gtsam::Symbol('P', i);
    Eigen::VectorXd poseNoise(6);
    poseNoise << 0.035679, 0.034466, 0.006353, 0.524538, 0.713080, 13.747008;
    auto poseNoiseModel = gtsam::noiseModel::Diagonal::Sigmas(poseNoise);
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(poseKey, trajectory[i],
                                               poseNoiseModel));
  }
}

// Function to initialize estimate values
gtsam::Values FingerprintPositionEstimator::initializeEstimateValues(
    gtsam::Key vehiclePositionKey, const Eigen::Vector3d &initialEstimate,
    const std::vector<Eigen::Vector3d> &positions,
    const std::vector<gtsam::Pose3> &trajectory) {
  gtsam::Values initialEstimateValues;
  initialEstimateValues.insert(
      vehiclePositionKey, gtsam::Point3(initialEstimate(0), initialEstimate(1),
                                        initialEstimate(2)));

  for (size_t i = 0; i < positions.size(); ++i) {
    gtsam::Key apKey = gtsam::Symbol('A', i);
    initialEstimateValues.insert(
        apKey,
        gtsam::Point3(positions[i].x(), positions[i].y(), positions[i].z()));
  }

  for (size_t i = 0; i < trajectory.size(); ++i) {
    gtsam::Key poseKey = gtsam::Symbol('P', i);
    initialEstimateValues.insert(poseKey, trajectory[i]);
  }

  std::cout << "Initial estimate: " << initialEstimate.transpose() << std::endl;
  initialEstimateValues.print("Initial Estimate Values:");
  return initialEstimateValues;
}

// Function to perform optimization
Eigen::Vector3d
FingerprintPositionEstimator::optimize(gtsam::NonlinearFactorGraph &graph,
                                       gtsam::Values &initialEstimateValues,
                                       gtsam::Key vehiclePositionKey) {
  try {
    gtsam::LevenbergMarquardtParams params;
    params.setMaxIterations(100);
    params.setRelativeErrorTol(1e-5);
    params.setAbsoluteErrorTol(1e-5);

    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initialEstimateValues,
                                                 params);
    gtsam::Values result = optimizer.optimize();

    gtsam::Point3 estimatedPosition =
        result.at<gtsam::Point3>(vehiclePositionKey);
    Eigen::Vector3d estimatedPosVec(
        estimatedPosition.x(), estimatedPosition.y(), estimatedPosition.z());

    std::cout << "Estimated position from optimizer: "
              << estimatedPosVec.transpose() << std::endl;
    return estimatedPosVec;
  } catch (const std::exception &e) {
    std::cerr << "Optimization failed: " << e.what() << std::endl;
    throw std::runtime_error("Optimization failed: " + std::string(e.what()));
  }
}

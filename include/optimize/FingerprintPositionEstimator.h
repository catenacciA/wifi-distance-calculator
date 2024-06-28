#ifndef FINGERPRINTPOSITIONESTIMATOR_H
#define FINGERPRINTPOSITIONESTIMATOR_H

#include <Eigen/Dense>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <map>
#include <string>
#include <vector>

#include "../../include/fingerprint/CSVDatabaseParser.h"
#include "../../include/fingerprint/FingerprintDatabase.h"
#include "../../include/fingerprint/FingerprintMatcher.h"
#include "../../include/fingerprint/RSSISimilarityCalculator.h"

class FingerprintPositionEstimator {
public:
  FingerprintPositionEstimator(const std::string &fingerprintFile,
                               const std::string &apFile);
  Eigen::Vector3d
  estimatePosition(const std::vector<RSSIData> &currentRSSIValues);

  const FingerprintDatabase &getFingerprintDatabase() const {
    return _fingerprintDatabase;
  }

private:
  CSVDatabaseParser _csvParser;
  std::vector<IWiFiScanner::APInfo> _fingerprintDB;
  std::map<std::string, std::pair<Eigen::Vector3d, Eigen::Matrix3d>> _apDatabase;
  RSSISimilarityCalculator _similarityCalculator;
  FingerprintMatcher _matcher;
  FingerprintDatabase _fingerprintDatabase;

  std::vector<gtsam::Pose3> parseLIOSLAMFile(const std::string &fileName);

  std::tuple<std::vector<Eigen::Vector3d>, std::vector<double>, std::vector<Eigen::Matrix3d>>
  prepareDataForOptimization(
      const std::vector<std::pair<std::string, Eigen::Vector3d>> &bestMatches,
      const Eigen::Vector3d &initialEstimate);

  void addPriorFactors(gtsam::NonlinearFactorGraph &graph,
                       gtsam::Key vehiclePositionKey,
                       const Eigen::Vector3d &initialEstimate);

  void addRangeFactors(gtsam::NonlinearFactorGraph &graph,
                       gtsam::Key vehiclePositionKey,
                       const std::vector<Eigen::Vector3d> &positions,
                       const std::vector<double> &distances,
                       const std::vector<Eigen::Matrix3d> &covarianceMatrices);

  void addPosePriors(gtsam::NonlinearFactorGraph &graph,
                     const std::vector<gtsam::Pose3> &trajectory);

  gtsam::Values
  initializeEstimateValues(gtsam::Key vehiclePositionKey,
                           const Eigen::Vector3d &initialEstimate,
                           const std::vector<Eigen::Vector3d> &positions,
                           const std::vector<gtsam::Pose3> &trajectory);

  Eigen::Vector3d optimize(gtsam::NonlinearFactorGraph &graph,
                           gtsam::Values &initialEstimateValues,
                           gtsam::Key vehiclePositionKey);
};

#endif // FINGERPRINTPOSITIONESTIMATOR_H

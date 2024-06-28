#include "../../include/fingerprint/FingerprintMatcher.h"
#include "../../include/fingerprint/LocationEstimator.h"
#include "../../include/fingerprint/RSSISimilarityCalculator.h"
#include "../../include/optimize/FingerprintPositionEstimator.h"
#include "../../include/wifi/WiFiScanner.h"
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>

int main(int argc, char *argv[]) {
  
  if (argc != 4) {
    std::cerr << "Usage: " << argv[0]
              << " <fingerprintFile> <apFile> <currentRSSIFile>" << std::endl;
    return -1;
  }

  std::string fingerprintFile = argv[1];
  std::string apFile = argv[2];
  std::string currentRSSIFile = argv[3];

  try {
    FingerprintPositionEstimator estimator(fingerprintFile, apFile);

    std::unique_ptr<ISimilarityCalculator> calculator =
        std::make_unique<RSSISimilarityCalculator>();

    std::unique_ptr<FingerprintMatcher> matcher =
        std::make_unique<FingerprintMatcher>(estimator.getFingerprintDatabase(),
                                             *calculator);

    LocationEstimator locationEstimator(std::move(matcher));

    auto currentRSSIValues =
        locationEstimator.readRSSIValuesFromFile(currentRSSIFile);
    Eigen::Vector3d estimatedPosition =
        estimator.estimatePosition(currentRSSIValues);

    std::cout << "Estimated Position: [" << estimatedPosition.x() << ", "
              << estimatedPosition.y() << ", " << estimatedPosition.z() << "]"
              << std::endl;

  } catch (const std::exception &e) {
    std::cerr << "Exception: " << e.what() << std::endl;
    return -1;
  }

  return 0;
}

#include "../../include/distance/LogDistanceCalculator.h"
#include "../../include/optimize/PositionEstimator.h"
#include "../../include/utility/ConfigParser.h"
#include "../../include/wifi/WiFiScanner.h"
#include <iostream>
#include <memory>

int main() {
  try {
    // Create the ConfigParser instance
    std::shared_ptr<IConfigParser> configParser =
        std::make_shared<ConfigParser>("../config.json");

    // Retrieve the AP parameters from the config
    const auto &apParameters = configParser->getAPParameters();

    // Create the distance calculator instance for each AP
    std::map<std::string, std::unique_ptr<IDistanceCalculator>>
        distanceCalculators;
    for (const auto &[bssid, params] : apParameters) {
      distanceCalculators[bssid] = std::make_unique<LogDistanceCalculator>(
          params.pathLossExponent, params.referenceDistance,
          params.referencePathLoss, params.transmitPower, params.sigma);
    }

    // Initialize the WiFi scanner with the map of distance calculators
    WiFiScanner scanner("wlp1s0f0", configParser,
                        std::move(distanceCalculators));

    // Initialize PositionEstimator
    PositionEstimator estimator;

    // Estimate the position
    Eigen::Vector3d estimatedPosition = estimator.estimatePosition(scanner);
    std::cout << "Estimated Position: " << estimatedPosition.transpose()
              << std::endl;
  } catch (const std::exception &e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}

#include "../../include/distance/LogDistanceCalculator.h"
#include "../../include/fingerprint/CSVDatabaseParser.h"
#include "../../include/fingerprint/FingerprintDatabase.h"
#include "../../include/fingerprint/FingerprintMatcher.h"
#include "../../include/fingerprint/LocationEstimator.h"
#include "../../include/fingerprint/RSSISimilarityCalculator.h"
#include "../../include/wifi/WiFiScanner.h"
#include <fstream>
#include <iostream>

void printUsage(const std::string &programName) {
  std::cout << "Usage: " << programName
            << " <WiFi_Interface> <Config_File> <Fingerprint_Data_File>"
            << std::endl;
  std::cout << "  <WiFi_Interface>: The name of the WiFi interface to use "
               "(e.g., wlan0)"
            << std::endl;
  std::cout << "  <Config_File>: The path to the configuration file (JSON)"
            << std::endl;
  std::cout << "  <Fingerprint_Data_File>: The path to the fingerprint data "
               "file (CSV)"
            << std::endl;
}

int main(int argc, char *argv[]) {
  if (argc < 4) {
    printUsage(argv[0]);
    return 1;
  }

  std::string wifiInterface = argv[1];
  std::string configFile = argv[2];
  std::string fingerprintDataFile = argv[3];

  try {
    auto distanceCalculator =
        std::make_unique<LogDistanceCalculator>(14.61, 1, 41.72, 39.40);
    auto wifiScanner = std::make_unique<WiFiScanner>(wifiInterface, std::move(distanceCalculator), configFile);

    CSVDatabaseParser csvParser;
    FingerprintDatabase database(csvParser, fingerprintDataFile);

    RSSISimilarityCalculator similarityCalculator;
    auto matcher =
        std::make_unique<FingerprintMatcher>(database, similarityCalculator);

    LocationEstimator estimator(std::move(wifiScanner), std::move(matcher));

    std::string estimatedLocation = estimator.estimateLocation();
    std::cout << "Estimated Location: " << estimatedLocation << std::endl;
  } catch (const std::exception &e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}

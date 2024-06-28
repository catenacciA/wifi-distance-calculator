#include "../../include/fingerprint/FingerprintDatabase.h"
#include <stdexcept>

// Constructor for the original database
FingerprintDatabase::FingerprintDatabase(IDatabaseParser &parser,
                                         const std::string &source) {
  database = parser.parse(source);
}

const std::map<std::string, std::map<std::string, double>> &
FingerprintDatabase::getDatabase() const {
  return database;
}

// New constructor for the extended database
FingerprintDatabase::FingerprintDatabase(
    const std::vector<IWiFiScanner::APInfo> &apInfos)
    : extendedDatabase(apInfos) {}

// New constructor for the AP database
FingerprintDatabase::FingerprintDatabase(
    const std::map<std::string, Eigen::Vector3d> &apPositions)
    : apDatabase(apPositions) {}

// New getter method for the extended database
const std::vector<IWiFiScanner::APInfo> &
FingerprintDatabase::getExtendedDatabase() const {
  return extendedDatabase;
}

// New getter method for the AP database
const std::map<std::string, Eigen::Vector3d> &
FingerprintDatabase::getAPDatabase() const {
  return apDatabase;
}

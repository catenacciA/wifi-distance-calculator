#ifndef CSVDATABASEPARSER_H
#define CSVDATABASEPARSER_H

#include "../wifi/IWiFiScanner.h"
#include "FingerprintDatabase.h"
#include <map>
#include <string>
#include <vector>

// Class for parsing CSV files to create a fingerprint database
class CSVDatabaseParser : public IDatabaseParser {
public:
  // Existing method to parse a different type of fingerprint database
  std::map<std::string, std::map<std::string, double>>
  parse(const std::string &csvFile) const override;

  // New method to parse a CSV database with headers: timestamp, mac, signalDBM,
  // ssid, x, y, z
  std::vector<IWiFiScanner::APInfo>
  parseExtendedFingerprintDatabase(const std::string &csvFile) const;

  // New method to parse a CSV database with known AP positions with headers:
  // ssid, mac, x, y, z, covariance
  std::map<std::string, std::pair<Eigen::Vector3d, Eigen::Matrix3d>>
  parseAPDatabase(const std::string &csvFile) const;
};

#endif // CSVDATABASEPARSER_H

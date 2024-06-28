#ifndef FINGERPRINTDATABASE_H
#define FINGERPRINTDATABASE_H

#include "../wifi/IWiFiScanner.h"
#include <map>
#include <string>
#include <vector>

// Interface for a database parser
class IDatabaseParser {
public:
  virtual ~IDatabaseParser() = default;
  virtual std::map<std::string, std::map<std::string, double>>
  parse(const std::string &source) const = 0;
};

// Class for storing a fingerprint database
class FingerprintDatabase {
public:
  FingerprintDatabase(IDatabaseParser &parser, const std::string &source);
  const std::map<std::string, std::map<std::string, double>> &
  getDatabase() const;

  // New constructors to handle different types of data
  FingerprintDatabase(const std::vector<IWiFiScanner::APInfo> &apInfos);
  
  FingerprintDatabase(
      const std::map<std::string, Eigen::Vector3d> &apPositions);

  // New getter methods for the different types of data
  const std::vector<IWiFiScanner::APInfo> &getExtendedDatabase() const;
  const std::map<std::string, Eigen::Vector3d> &getAPDatabase() const;

private:
  std::map<std::string, std::map<std::string, double>> database;
  std::vector<IWiFiScanner::APInfo> extendedDatabase;
  std::map<std::string, Eigen::Vector3d> apDatabase;
};

#endif // FINGERPRINTDATABASE_H

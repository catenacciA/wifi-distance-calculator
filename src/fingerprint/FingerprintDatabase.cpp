#include "../../include/fingerprint/FingerprintDatabase.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

FingerprintDatabase::FingerprintDatabase(const std::string &csvFile) {
  parseCSV(csvFile);
}

const std::map<std::string, std::map<std::string, double>> &
FingerprintDatabase::getDatabase() const {
  return database;
}

void FingerprintDatabase::parseCSV(const std::string &csvFile) {
  std::ifstream file(csvFile);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open fingerprint CSV file: " + csvFile);
  }

  std::string line;
  std::getline(file, line);

  while (std::getline(file, line)) {
    std::istringstream ss(line);
    std::string cellIdx, cellIdy, bssid, ssid, avgRssiStr;

    std::getline(ss, cellIdx, ',');
    std::getline(ss, cellIdy, ',');
    std::getline(ss, bssid, ',');
    std::getline(ss, ssid, ',');
    std::getline(ss, avgRssiStr, ',');

    std::string cellId = cellIdx + "," + cellIdy;

    try {
      double avgRssi = std::stod(avgRssiStr);
      database[cellId][bssid] = avgRssi;
    } catch (const std::invalid_argument &e) {
      std::cerr << "Invalid argument: unable to convert '" << avgRssiStr
                << "' to double. Line: " << line << std::endl;
    } catch (const std::out_of_range &e) {
      std::cerr << "Out of range: unable to convert '" << avgRssiStr
                << "' to double. Line: " << line << std::endl;
    }
  }
}

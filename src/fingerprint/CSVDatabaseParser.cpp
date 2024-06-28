#include "../../include/fingerprint/CSVDatabaseParser.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

// Helper function to split a string by delimiter
std::vector<std::string> split(const std::string &s, char delimiter) {
  std::vector<std::string> tokens;
  std::string token;
  bool inQuotes = false;
  std::stringstream tokenStream;

  for (char ch : s) {
    if (ch == '"') {
      inQuotes = !inQuotes;
    } else if (ch == delimiter && !inQuotes) {
      tokens.push_back(tokenStream.str());
      tokenStream.str("");
      tokenStream.clear();
    } else {
      tokenStream << ch;
    }
  }
  tokens.push_back(tokenStream.str());

  return tokens;
}

// Existing parse method implementation
std::map<std::string, std::map<std::string, double>>
CSVDatabaseParser::parse(const std::string &csvFile) const {
  std::map<std::string, std::map<std::string, double>> database;

  std::ifstream file(csvFile);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open fingerprint CSV file: " + csvFile);
  }

  std::string line;
  std::getline(file, line); // Skip header

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

  return database;
}

// New method to parse a CSV database with headers: timestamp, mac, signalDBM,
// ssid, x, y, z
std::vector<IWiFiScanner::APInfo>
CSVDatabaseParser::parseExtendedFingerprintDatabase(
    const std::string &csvFile) const {
  std::vector<IWiFiScanner::APInfo> apInfos;

  std::ifstream file(csvFile);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open fingerprint CSV file: " + csvFile);
  }

  std::string line;
  std::getline(file, line); // Skip header

  while (std::getline(file, line)) {
    auto tokens = split(line, ',');
    if (tokens.size() == 7) {
      IWiFiScanner::APInfo apInfo;
      apInfo.bssid = tokens[1];
      apInfo.signalStrength = std::stoi(tokens[2]);
      apInfo.ssid = tokens[3];
      apInfo.position[0] = std::stod(tokens[4]);
      apInfo.position[1] = std::stod(tokens[5]);
      apInfo.position[2] = std::stod(tokens[6]);
      // distance and seenMsAgo are not available in the CSV,
      // setting them to default values
      apInfo.distance = 0.0;
      apInfo.seenMsAgo = 0;
      apInfos.push_back(apInfo);
    }
  }
  file.close();
  return apInfos;
}

// Helper function to parse a flattened covariance matrix
Eigen::Matrix3d parseCovarianceMatrix(const std::string &covString) {
  // Remove the brackets
  std::string trimmed = covString.substr(1, covString.size() - 2);
  auto covTokens = split(trimmed, ','); // Use comma as delimiter here

  if (covTokens.size() != 9) {
    throw std::runtime_error("Invalid covariance matrix size");
  }

  Eigen::Matrix3d covariance;
  covariance << std::stod(covTokens[0]), std::stod(covTokens[1]),
      std::stod(covTokens[2]), std::stod(covTokens[3]), std::stod(covTokens[4]),
      std::stod(covTokens[5]), std::stod(covTokens[6]), std::stod(covTokens[7]),
      std::stod(covTokens[8]);
  return covariance;
}

// Modified method to parse a CSV database with known AP positions and
// covariance matrices with headers: ssid, mac, x, y, z, covariance
std::map<std::string, std::pair<Eigen::Vector3d, Eigen::Matrix3d>>
CSVDatabaseParser::parseAPDatabase(const std::string &csvFile) const {
  std::map<std::string, std::pair<Eigen::Vector3d, Eigen::Matrix3d>> apDatabase;
  std::ifstream file(csvFile);

  if (!file.is_open()) {
    throw std::runtime_error("Failed to open AP CSV file: " + csvFile);
  } else {
    std::cout << "Successfully opened AP CSV file: " << csvFile << std::endl;
  }

  std::string line;
  std::getline(file, line); // Skip header
  std::cout << "Skipping header line: " << line << std::endl;

  while (std::getline(file, line)) {
    std::cout << "Processing line: " << line << std::endl;
    auto tokens = split(line, ',');

    if (tokens.size() == 6) {
      Eigen::Vector3d position;
      position[0] = std::stod(tokens[2]);
      position[1] = std::stod(tokens[3]);
      position[2] = std::stod(tokens[4]);

      Eigen::Matrix3d covariance = parseCovarianceMatrix(tokens[5]);

      apDatabase[tokens[1]] = std::make_pair(position, covariance);

      std::cout << "Added AP: " << tokens[1]
                << " Position: " << position.transpose()
                << " Covariance: " << covariance << std::endl;
    } else {
      std::cout << "Invalid line format, expected 6 tokens but got "
                << tokens.size() << std::endl;
    }
  }
  file.close();
  std::cout << "Finished processing CSV file." << std::endl;
  return apDatabase;
}
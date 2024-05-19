#include "../../include/Logger.h"
#include "../../include/ProgressBar.h"
#include "../../include/distance/LogDistanceCalculator.h"
#include "../../include/wifi/WiFiScanner.h"
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <json/json.h>
#include <numeric>
#include <sstream>
#include <thread>
#include <vector>

Json::Value readGridConfig(const std::string &configFile) {
  std::ifstream file(configFile);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open configuration file: " +
                             configFile);
  }

  std::string content((std::istreambuf_iterator<char>(file)),
                      std::istreambuf_iterator<char>());

  Json::CharReaderBuilder builder;
  builder["collectComments"] = false;
  Json::Value config;
  std::string errs;

  std::istringstream sstr(content);
  if (!Json::parseFromStream(builder, sstr, &config, &errs)) {
    throw std::runtime_error("Failed to parse JSON configuration: " + errs);
  }

  return config;
}

void promptUserToMove(const std::string &currentPoint,
                      const std::string &nextPoint) {
  std::cout << "Data collection at reference point " << currentPoint
            << " complete." << std::endl;
  std::cout << "Please move to the next reference point: " << nextPoint
            << std::endl;
  std::cout << "Press Enter to continue..." << std::endl;
  std::cin.ignore();
  std::cin.get();
}

void printUsage(const std::string &programName) {
  std::cout << "Usage: " << programName
            << " <WiFi_Interface> <Config_File> <Output_File>" << std::endl;
  std::cout << "  <WiFi_Interface>: The name of the WiFi interface to use "
               "(e.g., wlan0)"
            << std::endl;
  std::cout << "  <Config_File>: The path to the grid configuration file (JSON)"
            << std::endl;
  std::cout << "  <Output_File>: The path to the output log file (CSV)"
            << std::endl;
}

std::tuple<double, double, int, int>
calculateStatistics(const std::vector<int> &data) {
  double mean = std::accumulate(data.begin(), data.end(), 0.0) / data.size();
  double accum = 0.0;
  std::for_each(data.begin(), data.end(),
                [&](const double d) { accum += (d - mean) * (d - mean); });
  double stdev = sqrt(accum / (data.size() - 1));
  int min = *std::min_element(data.begin(), data.end());
  int max = *std::max_element(data.begin(), data.end());
  return std::make_tuple(mean, stdev, min, max);
}

int main(int argc, char *argv[]) {
  if (argc < 4) {
    printUsage(argv[0]);
    return 1;
  }

  std::string wifiInterface = argv[1];
  std::string configFile = argv[2];
  std::string outputFile = argv[3];

  try {
    auto distanceCalculator =
        std::make_unique<LogDistanceCalculator>(14.61, 1, 41.72, 39.40);
    WiFiScanner wifiScanner(wifiInterface, std::move(distanceCalculator),
                            configFile);
    Logger logger;

    Json::Value config = readGridConfig(configFile);
    auto referencePoints = config["reference_points"];

    for (Json::Value::ArrayIndex i = 0; i < referencePoints.size(); ++i) {
      std::string currentPoint = std::to_string(referencePoints[i][0].asInt()) +
                                 "," +
                                 std::to_string(referencePoints[i][1].asInt());

      std::map<std::string, std::vector<int>> rssiData;

      for (int snapshot = 0; snapshot < 10; ++snapshot) {
        auto apInfos = wifiScanner.scan(true);
        for (const auto &apInfo : apInfos) {
          rssiData[apInfo.bssid].push_back(apInfo.signalStrength);
        }

        ProgressBar progressBar;
        progressBar.display(snapshot + 1, 10);

        std::this_thread::sleep_for(std::chrono::seconds(1));
      }

      std::map<std::string, std::tuple<double, double, int, int, double>>
          rssiStats;
      for (const auto &[bssid, rssiValues] : rssiData) {
        auto [mean, stdev, min, max] = calculateStatistics(rssiValues);
        double distance =
            wifiScanner.getDistanceCalculator()->calculateDistance(mean);
        rssiStats[bssid] = std::make_tuple(mean, stdev, min, max, distance);
      }

      logger.logData(currentPoint, rssiStats, outputFile);

      if (i < referencePoints.size() - 1) {
        std::string nextPoint =
            std::to_string(referencePoints[i + 1][0].asInt()) + "," +
            std::to_string(referencePoints[i + 1][1].asInt());
        promptUserToMove(currentPoint, nextPoint);
      } else {
        std::cout << "\nData collection complete at the final reference point "
                  << currentPoint << "." << std::endl;
      }
    }
  } catch (const std::exception &e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}

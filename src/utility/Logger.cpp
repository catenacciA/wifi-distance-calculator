#include "../include/utility/Logger.h"
#include <algorithm>
#include <cmath>
#include <fstream>

void Logger::logData(
    const std::string &cellId,
    const std::map<std::string, std::tuple<double, double, int, int, double>>
        &rssiStats,
    const std::string &filename) {
  std::ofstream logFile;
  bool fileExists = std::ifstream(filename).good();

  logFile.open(filename, std::ios_base::app);

  if (!fileExists) {
    logFile
        << "CellIDx,CellIDy,BSSID,AvgRSSI,StdDevRSSI,MinRSSI,MaxRSSI,Distance"
        << std::endl;
  }

  for (const auto &[bssid, stats] : rssiStats) {
    auto [mean, stdev, min, max, distance] = stats;
    logFile << cellId << "," << bssid << "," << mean << "," << stdev << ","
            << min << "," << max << "," << distance << std::endl;
  }
  logFile.close();
}

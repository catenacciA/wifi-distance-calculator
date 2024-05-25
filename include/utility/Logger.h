#ifndef LOGGER_H
#define LOGGER_H

#include <fstream>
#include <map>
#include <numeric>
#include <string>
#include <tuple>
#include <vector>

class ILogger {
public:
  virtual void logData(const std::string &cellId,
                       const std::map<std::string, std::tuple<double, double, int, int, double>> &rssiStats,
                       const std::string &filename) = 0;
  virtual ~ILogger() = default;
};

class Logger : public ILogger {
public:
  void logData(const std::string &cellId,
               const std::map<std::string, std::tuple<double, double, int, int, double>> &rssiStats,
               const std::string &filename) override;
};

#endif // LOGGER_H

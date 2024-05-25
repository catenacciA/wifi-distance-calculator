#ifndef ICONFIGPARSER_H
#define ICONFIGPARSER_H

#include <Eigen/Dense>
#include <map>
#include <string>
#include <vector>

class IConfigParser {
public:
  struct APParameters {
    double pathLossExponent;
    double referenceDistance;
    double referencePathLoss;
    double transmitPower;
    double sigma;
  };

  virtual const std::vector<std::string> &getTargetBSSIDs() const = 0;
  virtual const std::map<std::string, Eigen::Vector3d> &
  getAPPositions() const = 0;
  virtual const std::map<std::string, APParameters> &
  getAPParameters() const = 0;

  virtual ~IConfigParser() = default;
};

#endif // ICONFIGPARSER_H

#include "../../include/utility/ConfigParser.h"
#include <fstream>
#include <stdexcept>

ConfigParser::ConfigParser(const std::string &configFile) {
  parseConfig(configFile);
}

void ConfigParser::parseConfig(const std::string &configFile) {
  std::ifstream file(configFile, std::ifstream::binary);
  if (!file.is_open()) {
    throw std::runtime_error("Unable to open configuration file");
  }

  Json::Value root;
  file >> root;

  const Json::Value bssids = root["targetBSSIDs"];
  for (const auto &bssid : bssids) {
    targetBSSIDs.push_back(bssid.asString());
  }

  const Json::Value positions = root["ap_positions"];
  for (const auto &bssid : positions.getMemberNames()) {
    const Json::Value &position = positions[bssid];
    apPositions[bssid] = Eigen::Vector3d(
        position[0].asDouble(), position[1].asDouble(), position[2].asDouble());
  }

  const Json::Value parameters = root["ap_parameters"];
  for (const auto &bssid : parameters.getMemberNames()) {
    const Json::Value &param = parameters[bssid];
    APParameters apParams;
    apParams.pathLossExponent = param["pathLossExponent"].asDouble();
    apParams.referenceDistance = param["referenceDistance"].asDouble();
    apParams.referencePathLoss = param["referencePathLoss"].asDouble();
    apParams.transmitPower = param["transmitPower"].asDouble();
    apParams.sigma = param["sigma"].asDouble();
    apParameters[bssid] = apParams;
  }
}

const std::vector<std::string> &ConfigParser::getTargetBSSIDs() const {
  return targetBSSIDs;
}

const std::map<std::string, Eigen::Vector3d> &
ConfigParser::getAPPositions() const {
  return apPositions;
}

const std::map<std::string, IConfigParser::APParameters> &
ConfigParser::getAPParameters() const {
  return apParameters;
}

#ifndef CONFIGPARSER_H
#define CONFIGPARSER_H

#include "IConfigParser.h"
#include <json/json.h>

class ConfigParser : public IConfigParser {
public:
  explicit ConfigParser(const std::string &configFile);
  const std::vector<std::string> &getTargetBSSIDs() const override;
  const std::map<std::string, Eigen::Vector3d> &getAPPositions() const override;
  const std::map<std::string, APParameters> &getAPParameters() const override;

private:
  void parseConfig(const std::string &configFile);

  std::vector<std::string> targetBSSIDs;
  std::map<std::string, Eigen::Vector3d> apPositions;
  std::map<std::string, APParameters> apParameters;
};

#endif // CONFIGPARSER_H

// RSSIData.h

#ifndef RSSIDATA_H
#define RSSIDATA_H

#include <string>

struct RSSIData {
  std::string macAddress;
  double rssiValue;
  double mean;
  double std;
};

#endif // RSSIDATA_H

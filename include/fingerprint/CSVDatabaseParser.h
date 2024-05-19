#ifndef CSVDATABASEPARSER_H
#define CSVDATABASEPARSER_H

#include "../../include/fingerprint/FingerprintDatabase.h"
#include <string>
#include <map>

// Class for parsing CSV file to create a fingerprint database
class CSVDatabaseParser : public IDatabaseParser {
public:
    std::map<std::string, std::map<std::string, double>> parse(const std::string& csvFile) const override;
};

#endif // CSVDATABASEPARSER_H

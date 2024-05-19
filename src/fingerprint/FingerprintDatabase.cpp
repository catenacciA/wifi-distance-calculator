#include "../../include/fingerprint/FingerprintDatabase.h"
#include <stdexcept>

FingerprintDatabase::FingerprintDatabase(IDatabaseParser &parser,
                                         const std::string &source) {
  database = parser.parse(source);
}

const std::map<std::string, std::map<std::string, double>> &
FingerprintDatabase::getDatabase() const {
  return database;
}

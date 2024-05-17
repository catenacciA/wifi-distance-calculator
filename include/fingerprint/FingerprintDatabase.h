#ifndef FINGERPRINTDATABASE_H
#define FINGERPRINTDATABASE_H

#include <string>
#include <map>
#include <vector>

// Class for storing a fingerprint database
class FingerprintDatabase {
public:
    FingerprintDatabase(const std::string& csvFile);
    const std::map<std::string, std::map<std::string, double>>& getDatabase() const;

private:
    std::map<std::string, std::map<std::string, double>> database;
    void parseCSV(const std::string& csvFile);
};

#endif // FINGERPRINTDATABASE_H

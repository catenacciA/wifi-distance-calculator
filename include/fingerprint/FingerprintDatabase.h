#ifndef FINGERPRINTDATABASE_H
#define FINGERPRINTDATABASE_H

#include <string>
#include <map>

// Interface for a database parser
class IDatabaseParser {
public:
    virtual ~IDatabaseParser() = default;
    virtual std::map<std::string, std::map<std::string, double>> parse(const std::string& source) const = 0;
};

// Class for storing a fingerprint database
class FingerprintDatabase {
public:
    FingerprintDatabase(IDatabaseParser& parser, const std::string& source);
    const std::map<std::string, std::map<std::string, double>>& getDatabase() const;

private:
    std::map<std::string, std::map<std::string, double>> database;
};

#endif // FINGERPRINTDATABASE_H

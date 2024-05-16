#ifndef LOGDISTANCECALCULATOR_H
#define LOGDISTANCECALCULATOR_H

#include "DistanceCalculator.h"
#include <random>

// Class for calculating distance based on the log-distance path loss model
class LogDistanceCalculator : public DistanceCalculator {
public:
    LogDistanceCalculator(double pathLossExponent, double referenceDistance, double referenceRSSI, double sigma);
    double calculateDistance(int signalStrength) const override;

private:
    double pathLossExponent;
    double referenceDistance;
    double referenceRSSI;
    double sigma;

    mutable std::mt19937 gen;
    mutable std::normal_distribution<double> dist;
};

#endif // LOGDISTANCECALCULATOR_H

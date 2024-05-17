#ifndef DISTANCECALCULATOR_H
#define DISTANCECALCULATOR_H

// The DistanceCalculator class is an abstract class that defines the interface for all distance calculators.
class DistanceCalculator {
public:
    virtual ~DistanceCalculator() = default;
    virtual double calculateDistance(int signalStrength) const = 0;
};

#endif // DISTANCECALCULATOR_H
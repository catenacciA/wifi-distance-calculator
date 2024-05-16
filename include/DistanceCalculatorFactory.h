#ifndef DISTANCECALCULATORFACTORY_H
#define DISTANCECALCULATORFACTORY_H

#include "DistanceCalculator.h"
#include <memory>

// Factory class for creating DistanceCalculator objects
class DistanceCalculatorFactory {
public:
    static std::unique_ptr<DistanceCalculator> createLogDistanceCalculator();
};

#endif // DISTANCECALCULATORFACTORY_H

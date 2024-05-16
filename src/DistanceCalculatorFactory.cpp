#include "../include/DistanceCalculatorFactory.h"
#include "../include/LogDistanceCalculator.h"

std::unique_ptr<DistanceCalculator> DistanceCalculatorFactory::createLogDistanceCalculator() {
    double pathLossExponent = 14.61;
    double referenceDistance = 1;
    double referenceRSSI = 41.72;
    double sigma = 39.40;

    return std::make_unique<LogDistanceCalculator>(pathLossExponent, referenceDistance, referenceRSSI, sigma);
}

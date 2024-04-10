#ifndef DISTANCE_CALCULATOR_H
#define DISTANCE_CALCULATOR_H

class DistanceCalculator {
public:
  // Calculates the distance based on RSSI and a given transmitter power
  // (txPower). RSSI (Received Signal Strength Indicator) in dBm. txPower is the
  // known transmitted power of the access point at 1 meter in dBm. n is the
  // path loss exponent, which depends on the environment (range 2-4).
  static double calculateDistance(int rssi, int txPower, double n = 2.0);
};

#endif // DISTANCE_CALCULATOR_H

#ifndef WIFI_SCANNER_H
#define WIFI_SCANNER_H

#include <atomic>
#include <string>
#include <thread>
#include <vector>

class WiFiScanner {
public:
  WiFiScanner();
  ~WiFiScanner();

  void startScanning();
  void stopScanning();

  // Define the AccessPointInfo struct inside WiFiScanner
  struct AccessPointInfo {
    std::string SSID;
    float signalStrengthDbm;
    int frequencyMhz;

    AccessPointInfo(const std::string &ssid, float signalStrength,
                    int frequency)
        : SSID(ssid), signalStrengthDbm(signalStrength),
          frequencyMhz(frequency) {}
  };

private:
  std::atomic<bool> keepRunning;
  std::thread scannerThread;

  void scanForSignals();

  // Update the return type of parseScanResults to std::vector<AccessPointInfo>
  std::vector<AccessPointInfo> parseScanResults(const std::string &scanOutput);

  // Declare the isSSIDNull function inside WiFiScanner
  static bool isSSIDNull(const std::string &ssid);
};

#endif // WIFI_SCANNER_H

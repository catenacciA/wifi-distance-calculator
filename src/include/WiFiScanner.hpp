#ifndef WIFI_SCANNER_H
#define WIFI_SCANNER_H

#include <atomic>
#include <string>
#include <thread>
#include <vector>

/**
 * @brief This class provides functionalities to scan WiFi signals in the vicinity.
 * It encapsulates all the necessary details for starting and stopping the WiFi scanning process
 * and holds information about detected WiFi access points.
 */
class WiFiScanner {
public:
  /**
   * @brief Constructs a WiFiScanner object and initializes the scanning control mechanism.
   */
  WiFiScanner();

  /**
   * @brief Destroys the WiFiScanner object, ensuring that any ongoing scanning processes are properly stopped.
   */
  ~WiFiScanner();

  /**
   * @brief Starts the WiFi scanning process in a separate thread.
   */
  void startScanning();

  /**
   * @brief Stops the WiFi scanning process and joins the scanning thread.
   */
  void stopScanning();

  /**
   * @brief Represents information about a WiFi access point detected during scanning.
   */
  struct AccessPointInfo {
    std::string SSID;           ///< The Service Set Identifier (SSID) of the access point.
    float signalStrengthDbm;    ///< The signal strength of the access point in dBm.
    int frequencyMhz;           ///< The frequency of the access point in MHz.

    /**
     * @brief Constructs an AccessPointInfo object with specified SSID, signal strength, and frequency.
     * @param ssid The SSID of the WiFi access point.
     * @param signalStrength The signal strength in dBm.
     * @param frequency The frequency in MHz.
     */
    AccessPointInfo(const std::string &ssid, float signalStrength, int frequency)
        : SSID(ssid), signalStrengthDbm(signalStrength), frequencyMhz(frequency) {}
  };

private:
  std::atomic<bool> keepRunning;  ///< Controls the scanning process within the scanning thread.
  std::thread scannerThread;      ///< Thread object that runs the scanForSignals method.

  /**
   * @brief Continuously scans for WiFi signals while keepRunning is true.
   */
  void scanForSignals();

  /**
   * @brief Parses the raw scan results and constructs a vector of AccessPointInfo.
   * @param scanOutput The raw output from the scanning command or utility.
   * @return A vector of AccessPointInfo objects populated with parsed data.
   */
  std::vector<AccessPointInfo> parseScanResults(const std::string &scanOutput);
  
  /**
   * @brief Checks if the given SSID is considered null or empty.
   * @param ssid The SSID to check.
   * @return True if the SSID is null or empty, otherwise false.
   */
  static bool isSSIDNull(const std::string &ssid);
};

#endif // WIFI_SCANNER_H

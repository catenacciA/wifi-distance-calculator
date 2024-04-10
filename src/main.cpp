#include "include/WiFiScanner.hpp"
#include <chrono>
#include <thread>

int main() {
  // Create an instance of WiFiScanner
  WiFiScanner scanner;

  // Start scanning for WiFi signals
  scanner.startScanning();

  // Here we're assuming that the scanner will automatically print
  // the results to the console as part of the scanForSignals method
  // execution. This means we just need to give it some time to find networks.

  // Wait for an arbitrary amount of time to let the scanner pick up APs.
  // This is a simple synchronous way to wait. Depending on your application,
  // you might handle this asynchronously or trigger scanning in response
  // to some event.
  std::this_thread::sleep_for(std::chrono::seconds(10));

  // Stop scanning
  scanner.stopScanning();

  // The actual printing of AP information is handled within the
  // WiFiScanner::scanForSignals() method after parsing the scan results.

  return 0;
}

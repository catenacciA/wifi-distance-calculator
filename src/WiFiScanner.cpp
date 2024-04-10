#include "include/WiFiScanner.hpp"
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <unistd.h>
#include <vector>

// Constructor
WiFiScanner::WiFiScanner() : keepRunning(false) {}

// Destructor
WiFiScanner::~WiFiScanner() {
  if (scannerThread.joinable()) {
    stopScanning();
  }
}

// Start the scanning process in a separate thread
void WiFiScanner::startScanning() {
  keepRunning = true;
  scannerThread = std::thread([this] { this->scanForSignals(); });
}

// Signal the scanning thread to stop and wait for it to finish
void WiFiScanner::stopScanning() {
  keepRunning = false;
  if (scannerThread.joinable()) {
    scannerThread.join();
  }
}

void printResultsTable(
    const std::vector<WiFiScanner::AccessPointInfo> &results) {
  if (results.empty()) {
    std::cout << "No Access Points found." << std::endl;
    return;
  }

  // Print table header
  std::cout << std::left << std::setw(30) << "SSID" << std::setw(10) << "Signal"
            << std::setw(10) << "Freq" << std::endl;
  std::cout << std::string(50, '-') << std::endl; // Print a separator line

  // Print each AP's details
  for (const auto &ap : results) {
    std::cout << std::left << std::setw(30) << ap.SSID << std::setw(10)
              << ap.signalStrengthDbm << " dBm" << std::setw(10)
              << ap.frequencyMhz << " MHz" << std::endl;
  }
}

std::string findWiFiInterface() {
  FILE *pipe = popen("/usr/sbin/iw dev", "r");
  if (!pipe)
    throw std::runtime_error("popen() failed!");

  char buffer[128];
  std::string result = "";
  while (!feof(pipe)) {
    if (fgets(buffer, 128, pipe) != NULL)
      result += buffer;
  }
  int status = pclose(pipe);
  if (status == -1)
    throw std::runtime_error("pclose() failed!");

  std::istringstream iss(result);
  std::string line;
  while (std::getline(iss, line)) {
    std::size_t pos = line.find("Interface");
    if (pos != std::string::npos) {
      // Extract interface name
      std::istringstream issLine(line);
      std::string temp;
      issLine >> temp; // Skip "Interface" keyword
      issLine >> temp; // This should now contain the interface name
      return temp;
    }
  }

  throw std::runtime_error("Wi-Fi interface not found.");
}

void WiFiScanner::scanForSignals() {
  while (keepRunning.load()) {
    try {
      std::string interfaceName = findWiFiInterface();

      std::cout << "Scanning for wireless signals on interface: "
                << interfaceName << std::endl;

      char tempFilePath[] = "/tmp/wifi_scan_XXXXXX";
      int fd = mkstemp(tempFilePath);
      if (fd == -1) {
        std::cerr << "Failed to create temporary file for scan results."
                  << std::endl;
        return;
      }
      close(fd); // Close the file descriptor

      std::string command = "/usr/sbin/iw " + interfaceName +
                            " scan | grep -E 'SSID|signal:|freq:' > " +
                            tempFilePath;
      int result = system(command.c_str());

      if (result == 255) { // Typically indicates permission error
        std::cerr << "Failed to execute scan command. Permission denied."
                  << std::endl;
        std::remove(tempFilePath);
        return;
      } else if (result != 0) {
        std::cerr << "Command failed to execute." << std::endl;
        std::remove(tempFilePath);
        return;
      }

      // Parse the scan results from the temporary file
      auto parsedResults = parseScanResults(tempFilePath);

      // Print the results in a table
      printResultsTable(parsedResults);

      // Remove the temporary file
      std::remove(tempFilePath);
    } catch (const std::runtime_error &e) {
      std::cerr << "Error: " << e.what() << std::endl;
    }

    // Simulate scanning delay
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

bool WiFiScanner::isSSIDNull(const std::string &ssid) {
  if (ssid.empty()) {
    std::cerr << "DEBUG: SSID is empty." << std::endl;
    return true;
  }
  if (ssid.find("\x00") != std::string::npos) {
    std::cerr << "DEBUG: SSID contains null character." << std::endl;
    return true;
  }
  return false;
}

std::vector<WiFiScanner::AccessPointInfo>
WiFiScanner::parseScanResults(const std::string &filePath) {
  std::vector<AccessPointInfo> apInfos;
  std::ifstream fileStream(filePath);
  std::string line;

  if (!fileStream.is_open()) {
    std::cerr << "DEBUG: Failed to open file: " << filePath << std::endl;
    return apInfos;
  }

  std::string ssid;
  float signal = 0.0f;
  int freq = 0;

  while (std::getline(fileStream, line)) {
    std::cerr << "DEBUG: Processing line: " << line << std::endl;
    std::istringstream iss(line);
    std::string key;
    if (std::getline(iss, key, ':')) {
      std::string value;
      std::getline(iss >> std::ws, value); // Trim leading whitespace

      std::cerr << "DEBUG: Key=" << key << ", Value=" << value << std::endl;

      if (key == "freq") {
        freq = std::stoi(value);
        std::cerr << "DEBUG: Found frequency: " << freq << std::endl;
      } else if (key == "signal") {
        signal = std::stof(value.substr(0, value.find(" dBm")));
        std::cerr << "DEBUG: Found signal: " << signal << " dBm" << std::endl;
      } else if (key == "SSID") {
        ssid = value;
        if (!isSSIDNull(ssid)) { // Only add AP if SSID is not null
          std::cerr << "DEBUG: Found valid SSID: " << ssid << std::endl;
          apInfos.emplace_back(ssid, signal, freq);
        } else {
          std::cerr << "DEBUG: SSID is null, skipping." << std::endl;
        }
        // Reset variables for the next AP regardless of SSID validity
        ssid.clear();
        signal = 0.0f;
        freq = 0;
      }
    }
  }
  fileStream.close();
  std::cerr << "DEBUG: Finished parsing scan results. Found " << apInfos.size()
            << " access points." << std::endl;
  return apInfos;
}
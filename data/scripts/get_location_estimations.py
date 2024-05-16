import subprocess
import re
import sys
import argparse

def run_wifi_scanner(interface, config_file, fingerprint_data, num_runs, location_id, output_file):
    with open(output_file, 'w') as file:
        file.write("LocationID, x_estimated, y_estimated\n")
        
        for i in range(num_runs):
            try:
                result = subprocess.run(
                    ["sudo", "../../build/./wifi_scanner_online", interface, config_file, fingerprint_data],
                    capture_output=True,
                    text=True
                )

                if result.returncode == 0:
                    match = re.search(r'Estimated Location:\s*(\d+),(\d+)', result.stdout)
                    if match:
                        location = (int(match.group(1)), int(match.group(2)))
                        file.write(f"{location_id}, {location[0]}, {location[1]}\n")
                        print(f"Run {i+1}: Estimated Location: {location}")
                    else:
                        print(f"Run {i+1}: No estimated location found in output")
                else:
                    print(f"Run {i+1}: Command failed with return code {result.returncode}")
                    print(f"Error: {result.stderr}")
            except Exception as e:
                print(f"Run {i+1}: Exception occurred: {e}")

def main():
    parser = argparse.ArgumentParser(description="WiFi Scanner Script")
    parser.add_argument("location_id", type=str, help="Location ID for the scan")
    parser.add_argument("interface", type=str, help="WiFi interface to use for scanning")
    parser.add_argument("config_file", type=str, help="Path to the configuration file")
    parser.add_argument("fingerprint_data", type=str, help="Path to the fingerprint data file")
    parser.add_argument("num_runs", type=int, help="Number of runs to perform")
    parser.add_argument("output_file", type=str, help="Path to the output file")

    args = parser.parse_args()

    run_wifi_scanner(args.interface, args.config_file, args.fingerprint_data, args.num_runs, args.location_id, args.output_file)
    print(f"Estimated locations have been recorded in {args.output_file}")

if __name__ == "__main__":
    main()

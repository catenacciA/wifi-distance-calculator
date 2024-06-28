import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import glob
import os
from scipy.interpolate import interp1d

def load_trajectory_data(filepath):
    """Load trajectory data from a given file in TUM format."""
    trajectory_columns = ["timestamp", "x", "y", "z", "qx", "qy", "qz", "qw"]
    return pd.read_csv(filepath, sep=r"\s+", names=trajectory_columns)

def load_wifi_data(directory):
    """Load Wi-Fi scan data from files in a given directory."""
    wifi_files = sorted(glob.glob(f"{directory}/*.csv"), key=lambda x: os.path.basename(x))
    wifi_scans = [pd.read_csv(file) for file in wifi_files]
    wifi_timestamps = [
        float(os.path.basename(file).split("_")[-1].replace(".csv", "")) / 1e9 for file in wifi_files
    ]
    return wifi_scans, wifi_timestamps

def interpolate_trajectory(wifi_timestamps, trajectory_data):
    """Interpolate trajectory positions for each Wi-Fi timestamp."""
    interpolated_positions = []

    for axis in ['x', 'y', 'z']:
        interp_func = interp1d(trajectory_data['timestamp'], trajectory_data[axis], kind='cubic', fill_value="extrapolate")
        interpolated_positions.append(interp_func(wifi_timestamps))

    interpolated_positions = np.vstack(interpolated_positions).T
    return interpolated_positions

def extract_wifi_signals(wifi_scans):
    """Extract Wi-Fi signal information from scans."""
    return [scan[["mac", "signalDBM", "ssid"]] for scan in wifi_scans]

def create_fingerprinting_dataset(wifi_signals, wifi_timestamps, interpolated_positions):
    """Create Wi-Fi fingerprinting dataset."""
    fingerprinting_data = []
    for i, wifi_scan in enumerate(wifi_signals):
        timestamp = wifi_timestamps[i]
        position = interpolated_positions[i]
        for _, row in wifi_scan.iterrows():
            fingerprinting_data.append(
                [
                    timestamp,
                    row["mac"],
                    row["signalDBM"],
                    row["ssid"],
                    position[0],
                    position[1],
                    position[2],
                ]
            )
    fingerprinting_df = pd.DataFrame(
        fingerprinting_data,
        columns=["timestamp", "mac", "signalDBM", "ssid", "x", "y", "z"],
    )
    return fingerprinting_df

def estimate_ap_position(mac_address, wifi_signals, interpolated_positions):
    """Estimate AP position using weighted averaging with RSSI and calculate covariance matrix."""
    positions = []
    rssi_values = []
    ssid = None

    for i, wifi_scan in enumerate(wifi_signals):
        ap_data = wifi_scan[wifi_scan["mac"] == mac_address]
        if not ap_data.empty:
            rssi = ap_data["signalDBM"].mean()
            rssi_values.append(rssi)
            positions.append(interpolated_positions[i])
            ssid = ap_data["ssid"].iloc[0]

    if len(positions) == 0:
        return None, None, None

    positions = np.array(positions)
    rssi_values = np.array(rssi_values)
    weights = 10 ** (rssi_values / 10)
    weighted_position = np.average(positions, axis=0, weights=weights)
    covariance_matrix = np.cov(positions, rowvar=False, aweights=weights.flatten())

    return weighted_position, ssid, covariance_matrix

def estimate_all_ap_positions(wifi_signals, interpolated_positions):
    """Estimate positions for all APs."""
    ap_positions = {}
    ap_ssids = {}
    ap_covariances = {}
    all_aps = set(ap for scan in wifi_signals for ap in scan["mac"].unique())

    for ap in all_aps:
        position, ssid, covariance_matrix = estimate_ap_position(
            ap, wifi_signals, interpolated_positions
        )
        if position is not None:
            ap_positions[ap] = position
            ap_ssids[ap] = ssid
            ap_covariances[ap] = covariance_matrix

    return ap_positions, ap_ssids, ap_covariances

def create_ap_details_dataframe(ap_positions, ap_ssids, ap_covariances):
    """Create a DataFrame for the AP details including covariance matrices."""
    rows = []
    for mac, pos in ap_positions.items():
        row = {
            "SSID": ap_ssids[mac],
            "MAC": mac,
            "X": pos[0],
            "Y": pos[1],
            "Z": pos[2],
            "Covariance": ap_covariances[mac].flatten().tolist()
        }
        rows.append(row)
    return pd.DataFrame(rows).sort_values(by=["SSID", "MAC"])

def save_to_csv(df, filepath):
    """Save DataFrame to a CSV file."""
    df.to_csv(filepath, index=False)

def visualize_aps_and_trajectory(ap_details, trajectory_data):
    """Visualize the APs and Trajectory in 3D without grouping."""
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    x = trajectory_data["x"]
    y = trajectory_data["y"]
    z = trajectory_data["z"]
    ax.plot(x, y, z, label="Trajectory", alpha=0.5)

    ax.set_xlim([-12.5, 7.5])
    ax.set_ylim([-20.0, 10.0])
    ax.set_zlim([-10.0, 10.0])

    for _, row in ap_details.iterrows():
        ax.scatter(
            row["X"],
            row["Y"],
            row["Z"],
            label=f'{row["SSID"]} ({row["MAC"]})',
            marker="x",
        )

    ax.set_xlabel("X coordinate")
    ax.set_ylabel("Y coordinate")
    ax.set_zlabel("Z coordinate")
    ax.set_title("Estimated Positions of APs and Trajectory")

    ax.legend(loc="center left", bbox_to_anchor=(1, 0.5), fontsize="small")
    plt.show()

def evaluate_estimated_position(estimated_position, ap_details, wifi_signals, interpolated_positions):
    """Evaluate the estimated position by comparing it with the actual position using RSSI values."""
    mean_positions = {}
    for _, row in ap_details.iterrows():
        mean_positions[row["MAC"]] = np.array([row["X"], row["Y"], row["Z"]])

    estimated_pos = np.array([estimated_position["x"], estimated_position["y"], estimated_position["z"]])

    min_score = float('inf')
    best_idx = -1

    for i, interpolated_pos in enumerate(interpolated_positions):
        score = 0
        wifi_scan = wifi_signals[i]
        for ap, estimated_signal in estimated_position.items():
            if ap in mean_positions:
                actual_signal = wifi_scan[wifi_scan["mac"] == ap]["signalDBM"].mean()
                if not np.isnan(actual_signal):
                    score += abs(estimated_signal - actual_signal)
        if score < min_score:
            min_score = score
            best_idx = i

    actual_position = interpolated_positions[best_idx]
    distance = np.linalg.norm(actual_position - estimated_pos)
    
    signal_differences = {}
    closest_wifi_scan = wifi_signals[best_idx]
    for ap, estimated_signal in estimated_position.items():
        if ap in mean_positions:
            actual_signal = closest_wifi_scan[closest_wifi_scan["mac"] == ap]["signalDBM"].mean()
            signal_differences[ap] = abs(estimated_signal - actual_signal)
    return distance, signal_differences

def main():
    trajectory_data = load_trajectory_data("../input/lio_slam.txt")
    wifi_scans, wifi_timestamps = load_wifi_data("../input/wifi")

    if len(wifi_scans) >= len(trajectory_data):
        raise ValueError(
            "The number of Wi-Fi scans should be significantly less than the number of trajectory points."
        )

    interpolated_positions = interpolate_trajectory(wifi_timestamps, trajectory_data)
    wifi_signals = extract_wifi_signals(wifi_scans)

    fingerprinting_df = create_fingerprinting_dataset(
        wifi_signals, wifi_timestamps, interpolated_positions
    )

    save_to_csv(fingerprinting_df, "../output/map/wifi_fingerprinting_dataset.csv")

    ap_positions, ap_ssids, ap_covariances = estimate_all_ap_positions(
        wifi_signals, interpolated_positions
    )
    ap_details = create_ap_details_dataframe(ap_positions, ap_ssids, ap_covariances)

    save_to_csv(ap_details, "../output/map/ap_positions_ordered.csv")

    visualize_aps_and_trajectory(ap_details, trajectory_data)

    estimated_position = {
        "x": -6.86365024,
        "y": -9.6110357,
        "z": 0.761120703,
        "34:60:f9:2a:4:ae": 205.65,
        "4c:9e:ff:8e:2b:68": 191.56,
        "4e:68:ff:8e:2b:6c": 192.56
    }

    distance, signal_differences = evaluate_estimated_position(estimated_position, ap_details, wifi_signals, interpolated_positions)
    print(f"Distance between estimated position and actual position: {distance}")
    print(f"Signal differences: {signal_differences}")

if __name__ == "__main__":
    main()

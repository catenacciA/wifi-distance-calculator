import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from scipy.stats import skew, kurtosis, norm
from sklearn.preprocessing import MinMaxScaler
import glob

sns.set(style='whitegrid')
plt.rcParams.update({'figure.autolayout': True, 'font.size': 12})

palette = sns.color_palette('husl', 10)

def detect_outliers(df, column):
    Q1 = df[column].quantile(0.25)
    Q3 = df[column].quantile(0.75)
    IQR = Q3 - Q1
    lower_bound = Q1 - 1.5 * IQR
    upper_bound = Q3 + 1.5 * IQR
    return df[(df[column] < lower_bound) | (df[column] > upper_bound)]

def calculate_statistics(df, column):
    stats = {
        'mean': df[column].mean(),
        'std_dev': df[column].std(),
        'variance': df[column].var(),
        'skewness': skew(df[column]),
        'kurtosis': kurtosis(df[column])
    }
    return stats

def plot_gaussian_fit(ax, df, distance, color):
    mu, std = norm.fit(df['rssi'])
    sns.histplot(df['rssi'], bins=30, kde=False, stat='density', color=color, ax=ax, binwidth=1)
    xmin, xmax = df['rssi'].min(), df['rssi'].max()
    x = np.linspace(xmin, xmax, 100)
    p = norm.pdf(x, mu, std)
    ax.plot(x, p, 'k', linewidth=2)
    ax.set_title(f'Gaussian Fit (Dist: {distance}m)\nMean: {mu:.2f}, Std: {std:.2f}')
    ax.set_xlabel('RSSI')
    ax.set_ylabel('Density')

def plot_distribution(ax, df, distance, color):
    sns.histplot(df['rssi'], bins=30, kde=True, stat='density', color=color, ax=ax, binwidth=1)
    ax.axvline(df['rssi'].mean(), color='red', linestyle='--', label='Mean RSSI')
    ax.set_title(f'Distribution of RSSI (Dist: {distance}m)')
    ax.set_xlabel('RSSI')
    ax.set_ylabel('Density')
    ax.legend()

def plot_time_series(ax, df, distance, color):
    y_min = df['rssi'].min() - 5
    y_max = df['rssi'].max() + 5
    ax.plot(df['number_of_times'], df['rssi'], marker='o', linestyle='-', color=color, label='RSSI (dBm)')
    ax.axhline(y=df['rssi'].mean(), color='r', linestyle='--', label='Average RSSI')
    ax.set_ylim(y_min, y_max)
    ax.set_title(f'Time Series of RSSI (Dist: {distance}m)')
    ax.set_xlabel('Measurement Times')
    ax.set_ylabel('RSSI (dBm)')
    ax.legend()

def calculate_model_parameters(df, distances, output_dir):
    A = abs(df[df['distance'] == 1]['rssi'].mean())
    path_loss_indices = {}
    
    with open(os.path.join(output_dir, 'model_parameters.txt'), 'w') as f:
        f.write(f'A (Radio frequency parameter at 1m): {A} dBm\n\n')
        f.write('Path loss indices:\n')
        
        for d in distances:
            if d == 1:
                continue
            mean_rssi = df[df['distance'] == d]['rssi'].mean()
            n_d = (A - mean_rssi) / (10 * np.log10(d))
            path_loss_indices[d] = n_d
            f.write(f'n_d at distance {d}m: {n_d}\n')
        
        mean_n = np.mean(list(path_loss_indices.values()))
        f.write(f'\nMean path loss index n: {mean_n}')
    
    return A, path_loss_indices, mean_n

base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../'))
input_dir = os.path.join(base_dir, 'data/input/rssi_logs')
output_base_dir = os.path.join(base_dir, 'data/output/rssi_analysis')
os.makedirs(output_base_dir, exist_ok=True)

file_pattern = os.path.join(input_dir, "rssi_log_at_*.csv")
file_paths = glob.glob(file_pattern)

data = []

for file_path in file_paths:
    distance = int(file_path.split('_')[-1].split('.')[0])
    df = pd.read_csv(file_path)
    df['distance'] = distance
    data.append(df)

combined_data = pd.concat(data, ignore_index=True)

scaler = MinMaxScaler()
combined_data['rssi_normalized'] = scaler.fit_transform(combined_data[['rssi']])

descriptive_stats = combined_data[['rssi', 'rssi_normalized']].describe()

grouped_stats = combined_data.groupby('distance')[['rssi', 'rssi_normalized']].describe().unstack()

# Calculate combined statistics for the entire dataset
combined_statistics = calculate_statistics(combined_data, 'rssi')
combined_stats_df = pd.DataFrame([combined_statistics])
combined_stats_df.to_csv(os.path.join(output_base_dir, 'combined_statistics.csv'), index=False)

# Add combined statistics to grouped statistics
combined_stats_series = pd.Series(combined_statistics, name='combined')
grouped_stats = grouped_stats._append(combined_stats_series)

descriptive_stats.to_csv(os.path.join(output_base_dir, 'descriptive_statistics.csv'))
grouped_stats.to_csv(os.path.join(output_base_dir, 'grouped_statistics.csv'))

individual_stats = {}
distances = combined_data['distance'].unique()

fig_time_series, axs_time_series = plt.subplots((len(distances) + 1) // 2, 2, figsize=(15, 15))
fig_distribution, axs_distribution = plt.subplots((len(distances) + 1) // 2, 2, figsize=(15, 15))
fig_gaussian_fit, axs_gaussian_fit = plt.subplots((len(distances) + 1) // 2, 2, figsize=(15, 15))

with open(os.path.join(output_base_dir, 'individual_statistics.txt'), 'w') as stats_file:
    for idx, distance in enumerate(distances):
        df_distance = combined_data[combined_data['distance'] == distance]
        distance_stats = calculate_statistics(df_distance, 'rssi')
        individual_stats[distance] = distance_stats
        
        stats_file.write(f"Distance: {distance}m\n")
        for key, value in distance_stats.items():
            stats_file.write(f'{key}: {value}\n')
        stats_file.write('\n')
        
        plot_gaussian_fit(axs_gaussian_fit.flat[idx], df_distance, distance, palette[idx])
        
        plot_distribution(axs_distribution.flat[idx], df_distance, distance, palette[idx])
        
        plot_time_series(axs_time_series.flat[idx], df_distance, distance, palette[idx])

fig_time_series.tight_layout()
fig_distribution.tight_layout()
fig_gaussian_fit.tight_layout()
fig_time_series.savefig(os.path.join(output_base_dir, 'time_series_plots.png'), dpi=300)
fig_distribution.savefig(os.path.join(output_base_dir, 'distribution_plots.png'), dpi=300)
fig_gaussian_fit.savefig(os.path.join(output_base_dir, 'gaussian_fit_plots.png'), dpi=300)

A, path_loss_indices, mean_n = calculate_model_parameters(combined_data, distances, output_base_dir)

short_distance = combined_data[combined_data['distance'].isin([1, 2, 3])]
medium_distance = combined_data[combined_data['distance'].isin([4, 5, 6, 7])]
long_distance = combined_data[combined_data['distance'].isin([8, 9, 10])]

grouped_stats = {
    'short_distance': calculate_statistics(short_distance, 'rssi'),
    'medium_distance': calculate_statistics(medium_distance, 'rssi'),
    'long_distance': calculate_statistics(long_distance, 'rssi')
}

grouped_output_dir = os.path.join(output_base_dir, 'grouped_statistics')
os.makedirs(grouped_output_dir, exist_ok=True)
grouped_stats_df = pd.DataFrame(grouped_stats)
grouped_stats_df.to_csv(os.path.join(grouped_output_dir, 'grouped_statistics.csv'))

fig_grouped_dist, axs_grouped_dist = plt.subplots(1, 3, figsize=(18, 6), constrained_layout=True)
plot_distribution(axs_grouped_dist[0], short_distance, 'Short Distances (1-3)', palette[0])
plot_distribution(axs_grouped_dist[1], medium_distance, 'Medium Distances (4-7)', palette[1])
plot_distribution(axs_grouped_dist[2], long_distance, 'Long Distances (8-10)', palette[2])
fig_grouped_dist.savefig(os.path.join(grouped_output_dir, 'grouped_distribution.png'), dpi=300)

print("Analysis complete. Results saved in 'data/output/rssi_analysis' directory.")

import os
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
import re

input_path = '../input/distance_estimates/'
output_path = '../output/errors_logDistance/'

os.makedirs(output_path, exist_ok=True)

def extract_distance(filename):
    match = re.search(r'\d+', filename)
    if match:
        return int(match.group())
    else:
        raise ValueError(f"Cannot extract distance from filename: {filename}")

files = [f for f in os.listdir(input_path) if f.startswith('measured_distance_at_') and f.endswith('.csv')]

dataframes = []
for file in files:
    df = pd.read_csv(os.path.join(input_path, file))
    df['reference_distance'] = extract_distance(file)
    dataframes.append(df)

data = pd.concat(dataframes, ignore_index=True)

data['absolute_error'] = np.abs(data['distance (m)'] - data['reference_distance'])
data['relative_error'] = data['absolute_error'] / data['reference_distance']
data['squared_error'] = data['absolute_error'] ** 2

short_distances = data[data['reference_distance'].between(1, 3)]
middle_distances = data[data['reference_distance'].between(4, 7)]
long_distances = data[data['reference_distance'].between(8, 10)]

def calculate_metrics(df):
    mae = np.mean(df['absolute_error'])
    rmse = np.sqrt(np.mean(df['squared_error']))
    return pd.Series({'MAE': mae, 'RMSE': rmse})

all_stats = data[['absolute_error', 'relative_error']].describe()
all_metrics = calculate_metrics(data)

short_stats = short_distances[['absolute_error', 'relative_error']].describe()
short_metrics = calculate_metrics(short_distances)

middle_stats = middle_distances[['absolute_error', 'relative_error']].describe()
middle_metrics = calculate_metrics(middle_distances)

long_stats = long_distances[['absolute_error', 'relative_error']].describe()
long_metrics = calculate_metrics(long_distances)

stats_combined = pd.concat([
    all_stats, all_metrics,
    short_stats, short_metrics,
    middle_stats, middle_metrics,
    long_stats, long_metrics
], axis=1, keys=[
    'All Distances', 'All Metrics',
    'Short Distances', 'Short Metrics',
    'Middle Distances', 'Middle Metrics',
    'Long Distances', 'Long Metrics'
])

stats_combined = stats_combined.round(4)
stats_combined.to_csv(os.path.join(output_path, 'summary_statistics.csv'))

sns.set(style="whitegrid")
unique_distances = data['reference_distance'].unique()
unique_distances.sort()
n = len(unique_distances)
fig, axes = plt.subplots(n // 2 + n % 2, 2, figsize=(14, 5 * (n // 2)))
axes = axes.flatten()

for i, distance in enumerate(unique_distances):
    subset = data[data['reference_distance'] == distance]
    sns.histplot(subset['distance (m)'] - distance, kde=True, ax=axes[i], color='gray')
    axes[i].axvline(0, color='red', linestyle='--', label='Zero Error Line')
    axes[i].set_title(f'Error Distribution at {distance} m')
    axes[i].set_xlabel('Error (m)')
    axes[i].set_ylabel('Frequency')
    axes[i].legend()

plt.tight_layout()
plt.savefig(os.path.join(output_path, 'error_distributions.png'), dpi=300)
plt.close()

fig, axes = plt.subplots(2, 2, figsize=(14, 12))
fig.suptitle('Distribution of Absolute Errors')

sns.histplot(data['absolute_error'], kde=True, ax=axes[0, 0], color='blue')
axes[0, 0].set_title('All Distances')
axes[0, 0].set_xlabel('Absolute Error')

sns.histplot(short_distances['absolute_error'], kde=True, ax=axes[0, 1], color='green')
axes[0, 1].set_title('Short Distances (1-3 meters)')
axes[0, 1].set_xlabel('Absolute Error')

sns.histplot(middle_distances['absolute_error'], kde=True, ax=axes[1, 0], color='orange')
axes[1, 0].set_title('Middle Distances (4-7 meters)')
axes[1, 0].set_xlabel('Absolute Error')

sns.histplot(long_distances['absolute_error'], kde=True, ax=axes[1, 1], color='red')
axes[1, 1].set_title('Long Distances (8-10 meters)')
axes[1, 1].set_xlabel('Absolute Error')

plt.tight_layout(rect=[0, 0, 1, 0.96])
plt.savefig(os.path.join(output_path, 'absolute_error_distribution.png'), dpi=300)
plt.close()

fig, axes = plt.subplots(2, 2, figsize=(14, 12))
fig.suptitle('Distribution of Relative Errors')

sns.histplot(data['relative_error'], kde=True, ax=axes[0, 0], color='blue')
axes[0, 0].set_title('All Distances')
axes[0, 0].set_xlabel('Relative Error')

sns.histplot(short_distances['relative_error'], kde=True, ax=axes[0, 1], color='green')
axes[0, 1].set_title('Short Distances (1-3 meters)')
axes[0, 1].set_xlabel('Relative Error')

sns.histplot(middle_distances['relative_error'], kde=True, ax=axes[1, 0], color='orange')
axes[1, 0].set_title('Middle Distances (4-7 meters)')
axes[1, 0].set_xlabel('Relative Error')

sns.histplot(long_distances['relative_error'], kde=True, ax=axes[1, 1], color='red')
axes[1, 1].set_title('Long Distances (8-10 meters)')
axes[1, 1].set_xlabel('Relative Error')

plt.tight_layout(rect=[0, 0, 1, 0.96])
plt.savefig(os.path.join(output_path, 'relative_error_distribution.png'), dpi=300)
plt.close()

error_trends = data.groupby('reference_distance').agg({
    'absolute_error': ['mean', 'std'],
    'relative_error': ['mean', 'std'],
    'squared_error': 'mean'
}).reset_index()

error_trends.columns = ['reference_distance', 'mean_absolute_error', 'std_absolute_error', 'mean_relative_error', 'std_relative_error', 'MSE']

error_trends['RMSE'] = np.sqrt(error_trends['MSE'])

fig, axes = plt.subplots(3, 1, figsize=(14, 18))

sns.lineplot(x='reference_distance', y='mean_absolute_error', data=error_trends, ax=axes[0], marker='o', label='Mean Absolute Error', color='red')
sns.lineplot(x='reference_distance', y='mean_relative_error', data=error_trends, ax=axes[0], marker='o', label='Mean Relative Error', color='blue')
axes[0].set_title('Mean Errors by Reference Distance')
axes[0].set_xlabel('Reference Distance (m)')
axes[0].set_ylabel('Mean Error')
axes[0].legend()

sns.lineplot(x='reference_distance', y='std_absolute_error', data=error_trends, ax=axes[1], marker='o', label='Standard Deviation of Absolute Error', color='red')
sns.lineplot(x='reference_distance', y='std_relative_error', data=error_trends, ax=axes[1], marker='o', label='Standard Deviation of Relative Error', color='blue')
axes[1].set_title('Standard Deviation of Errors by Reference Distance')
axes[1].set_xlabel('Reference Distance (m)')
axes[1].set_ylabel('Standard Deviation of Error')
axes[1].legend()

sns.lineplot(x='reference_distance', y='mean_absolute_error', data=error_trends, ax=axes[2], marker='o', label='MAE', color='red')
sns.lineplot(x='reference_distance', y='RMSE', data=error_trends, ax=axes[2], marker='o', label='RMSE', color='green')
axes[2].set_title('MAE and RMSE by Reference Distance')
axes[2].set_xlabel('Reference Distance (m)')
axes[2].set_ylabel('Error')
axes[2].legend()

plt.tight_layout()
plt.savefig(os.path.join(output_path, 'error_trends.png'), dpi=300)
plt.close()

print("Analysis complete. Results saved in 'data/output/errors_logDistance' directory.")

import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

output_dir = "../output/location_errors/visualizations"
os.makedirs(output_dir, exist_ok=True)

file_paths = {
    '3_7': '../output/location_errors/estimations_at_(3,7).csv',
    '3_7_v2': '../output/location_errors/estimation_at_(3,7)v2.csv',
    '4_3': '../output/location_errors/estimations_at_(4,3).csv',
    '5_5': '../output/location_errors/estimations_at_(5,5).csv',
    '6_3': '../output/location_errors/estimations_at_(6,3).csv'
}

datasets = {}
true_locations = {
    '3_7': np.array([3, 7]),
    '3_7_v2': np.array([3, 7]),
    '4_3': np.array([4, 3]),
    '5_5': np.array([5, 5]),
    '6_3': np.array([6, 3])
}

for key, path in file_paths.items():
    df = pd.read_csv(path)
    df.columns = df.columns.str.strip()
    datasets[key] = df

combined_estimations = pd.concat(datasets.values())
true_locations_combined = np.concatenate(
    [np.tile(loc, (len(datasets[key]), 1)) for key, loc in true_locations.items()],
    axis=0
)

combined_estimations['error'] = np.linalg.norm(combined_estimations[['x_estimated', 'y_estimated']].values - true_locations_combined, axis=1)
combined_errors = combined_estimations['error']
combined_mean_estimation = combined_estimations[['x_estimated', 'y_estimated']].mean().values
combined_std_estimation = combined_estimations[['x_estimated', 'y_estimated']].std().values
combined_mean_error = np.mean(combined_errors)
combined_median_error = np.median(combined_errors)
combined_rmse = np.sqrt(np.mean(combined_errors**2))

combined_results = {
    'mean_estimation': combined_mean_estimation,
    'std_estimation': combined_std_estimation,
    'mean_error': combined_mean_error,
    'median_error': combined_median_error,
    'rmse': combined_rmse
}

fig, axs = plt.subplots(len(datasets), 2, figsize=(15, 25))
fig.suptitle('Error Trends and Distribution of Estimations')
colors = sns.color_palette("husl", len(datasets))
results = {}

for i, (key, df) in enumerate(datasets.items()):
    true_location = true_locations[key]
    errors = np.linalg.norm(df[['x_estimated', 'y_estimated']].values - true_location, axis=1)
    
    mean_estimation = df[['x_estimated', 'y_estimated']].mean().values
    std_estimation = df[['x_estimated', 'y_estimated']].std().values
    mean_error = np.mean(errors)
    median_error = np.median(errors)
    rmse = np.sqrt(np.mean(errors**2))
    
    results[key] = {
        'mean_estimation': mean_estimation,
        'std_estimation': std_estimation,
        'mean_error': mean_error,
        'median_error': median_error,
        'rmse': rmse
    }

    ax = axs[i, 0]
    ax.plot(errors, marker='o', linestyle='-', color=colors[i])
    ax.set_title(f'Error Trend for Location {key}')
    ax.set_xlabel('Estimation Index')
    ax.set_ylabel('Error (meters)')
    ax.grid(True)
    
    ax = axs[i, 1]
    sns.histplot(errors, bins=10, color=colors[i], ax=ax, kde=True)
    ax.set_title(f'Distribution of Errors for Location {key}')
    ax.set_xlabel('Error (meters)')
    ax.set_ylabel('Frequency')
    ax.grid(True)

plt.tight_layout(rect=[0, 0, 1, 0.96])
error_trends_path = os.path.join(output_dir, 'error_trends_and_distributions.png')
plt.savefig(error_trends_path)
plt.close()

combined_errors_df = pd.DataFrame({
    'Errors': combined_errors,
    'Location': np.repeat(list(true_locations.keys()), [len(datasets[key]) for key in true_locations.keys()])
})

# Box Plot of Errors by Location
fig, ax = plt.subplots(figsize=(12, 8))
sns.boxplot(data=combined_errors_df, x='Location', y='Errors', hue='Location', palette="husl", ax=ax, dodge=False)
ax.set_title('Box Plot of Errors by Location')
ax.set_xlabel('Location')
ax.set_ylabel('Error (meters)')
box_plot_path = os.path.join(output_dir, 'box_plot_errors.png')
plt.tight_layout()
plt.savefig(box_plot_path)
plt.close()

# Scatter Plots of Estimations vs True Locations
fig, axs = plt.subplots(len(datasets), 1, figsize=(10, len(datasets) * 5))
for i, (key, df) in enumerate(datasets.items()):
    true_location = true_locations[key]
    axs[i].scatter(df['x_estimated'], df['y_estimated'], label=key, color=colors[i])
    axs[i].scatter(true_location[0], true_location[1], color='black', marker='x', label='True Location')
    axs[i].set_title(f'Scatter Plot of Estimated vs True Locations for {key}')
    axs[i].set_xlabel('X Estimated')
    axs[i].set_ylabel('Y Estimated')
    axs[i].legend()
scatter_plot_path = os.path.join(output_dir, 'scatter_plot_estimations.png')
plt.tight_layout()
plt.savefig(scatter_plot_path)
plt.close()

# Heatmaps of Estimation Errors
fig, axs = plt.subplots(len(datasets), 1, figsize=(10, len(datasets) * 5))
for i, (key, df) in enumerate(datasets.items()):
    df['error'] = np.linalg.norm(df[['x_estimated', 'y_estimated']].values - true_locations[key], axis=1)
    heatmap_data = pd.pivot_table(df, values='error', index='y_estimated', columns='x_estimated', aggfunc='mean').fillna(0)
    sns.heatmap(heatmap_data, cmap='coolwarm', ax=axs[i])
    axs[i].set_title(f'Heatmap of Estimation Errors for {key}')
heatmap_path = os.path.join(output_dir, 'heatmap_estimation_errors.png')
plt.tight_layout()
plt.savefig(heatmap_path)
plt.close()

# Violin Plot of Errors by Location
fig, ax = plt.subplots(figsize=(12, 8))
sns.violinplot(data=combined_errors_df, x='Location', y='Errors', hue='Location', palette="husl", ax=ax, dodge=False)
ax.set_title('Violin Plot of Errors by Location')
ax.set_xlabel('Location')
ax.set_ylabel('Error (meters)')
violin_plot_path = os.path.join(output_dir, 'violin_plot_errors.png')
plt.tight_layout()
plt.savefig(violin_plot_path)
plt.close()

# Cumulative Distribution Function (CDF) of Errors
fig, ax = plt.subplots(figsize=(10, 6))
for key, df in datasets.items():
    true_location = true_locations[key]
    errors = np.linalg.norm(df[['x_estimated', 'y_estimated']].values - true_location, axis=1)
    sns.ecdfplot(errors, ax=ax, label=key)
ax.set_title('Cumulative Distribution Function (CDF) of Errors')
ax.set_xlabel('Error (meters)')
ax.set_ylabel('Cumulative Probability')
ax.legend()
cdf_plot_path = os.path.join(output_dir, 'cdf_plot_errors.png')
plt.tight_layout()
plt.savefig(cdf_plot_path)
plt.close()

# Correlation Matrices of Estimations
fig, axs = plt.subplots(len(datasets), 1, figsize=(12, len(datasets) * 5))
for i, (key, df) in enumerate(datasets.items()):
    correlation_matrix = df.corr()
    sns.heatmap(correlation_matrix, annot=True, cmap='coolwarm', ax=axs[i])
    axs[i].set_title(f'Correlation Matrix of Estimations for {key}')
correlation_matrix_path = os.path.join(output_dir, 'correlation_matrices.png')
plt.tight_layout()
plt.savefig(correlation_matrix_path)
plt.close()

metrics_path = os.path.join(output_dir, 'error_and_statistical_metrics.txt')
with open(metrics_path, 'w') as f:
    for key, metrics in results.items():
        f.write(f"Results for {key}:\n")
        f.write(f"Mean Estimated Location: {metrics['mean_estimation']}\n")
        f.write(f"Standard Deviation: {metrics['std_estimation']}\n")
        f.write(f"Mean Error: {metrics['mean_error']} meters\n")
        f.write(f"Median Error: {metrics['median_error']} meters\n")
        f.write(f"RMSE: {metrics['rmse']} meters\n")
        f.write("\n")
    
    f.write("Combined Results:\n")
    f.write(f"Mean Estimated Location: {combined_results['mean_estimation']}\n")
    f.write(f"Standard Deviation: {combined_results['std_estimation']}\n")
    f.write(f"Mean Error: {combined_results['mean_error']} meters\n")
    f.write(f"Median Error: {combined_results['median_error']} meters\n")
    f.write(f"RMSE: {combined_results['rmse']} meters\n")

print(f"Visualizations and metrics saved in directory: {output_dir}")

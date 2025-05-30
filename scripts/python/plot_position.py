# Implemented by: Nico
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import load_data as ld


# Define file paths
truth_path = "controllers/supervisor/data/ground_truth.csv"
odo_path = "controllers/controller/data/odo.csv"
sigma_path = "controllers/controller/data/odo_sigma.csv"

# Load data
ground_truth = ld.load_file(truth_path)
odometry = ld.load_file(odo_path)
uncertainty = ld.load_file(sigma_path)

# Extract pose
gt_x = ground_truth['x'].to_numpy()
gt_y = ground_truth['y'].to_numpy()
gt_heading = ground_truth['heading'].to_numpy()

odo_x = odometry['x'].to_numpy()
odo_y = odometry['y'].to_numpy()
odo_heading = odometry['heading'].to_numpy()

# Figure 1: Ground Truth vs Odometry
plt.figure(figsize=(10, 6))
plt.plot(gt_x, gt_y, label='Ground Truth', color='blue', linewidth=2)
plt.plot(odo_x, odo_y, label='Odometry Estimate', color='orange', linestyle='--', linewidth=2)
plt.title('Ground Truth vs Odometry Trajectory')
plt.xlabel('X Position [m]')
plt.ylabel('Y Position [m]')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.tight_layout()
plt.show()

""" x and y error """
# Merge on 'time' to align matching timestamps
merged_all = pd.merge(odometry, ground_truth, on='time', suffixes=('_odo', '_gt'))
time = merged_all['time'].to_numpy()

# Compute x and y errors
error_x = (merged_all['x_odo'] - merged_all['x_gt']).abs().to_numpy()
error_y = (merged_all['y_odo'] - merged_all['y_gt']).abs().to_numpy()

# Uncertainty in x and y
sigma2_x = uncertainty['x'].to_numpy()
sigma2_y = uncertainty['y'].to_numpy()

sigma_x = np.sqrt(sigma2_x)
sigma_y = np.sqrt(sigma2_y)

# Figure 3: X and Y error
plt.figure(figsize=(12, 6))
plt.plot(time, error_x, label='X Error', color='blue')
plt.plot(time, sigma_x, label='X Uncertainty', color='blue', linestyle='--')
plt.plot(time, error_y, label='Y Error', color='orange')
plt.plot(time, sigma_y, label='Y Uncertainty', color='orange', linestyle='--')
plt.title('Position Error vs Uncertainty')
plt.xlabel('Time [s]')
plt.ylabel('Error [m]')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

# Calculate angular error in rad
heading_diff = (merged_all['heading_odo'] - merged_all['heading_gt']) % (2 * np.pi)
heading_error = np.min((heading_diff, 2 * np.pi - heading_diff), axis=0)

heading_error *= 180 / np.pi # Convert to degrees

sigma2_heading = uncertainty['heading'].to_numpy()
sigma_heading = np.sqrt(sigma2_heading)
sigma_heading *= 180 / np.pi # Convert to degrees

# Figure 3: Heading error
plt.figure(figsize=(12, 6))
plt.plot(time, heading_error, label='Heading Error', color='dodgerblue')
plt.plot(time, sigma_heading, label='Heading Uncertainty', color='dodgerblue', linestyle='--')
plt.title('Heading Error vs Uncertainty')
plt.xlabel('Time [s]')
plt.ylabel('Heading Error [$\degree$]')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
import pandas as pd
import matplotlib.pyplot as plt
import load_data as ld
import numpy as np

# Load ground truth trajectory
filename = "controllers/supervisor/data/ground_truth.csv"
data = ld.load_file(filename)
print(data.head(6))
# Load collisions
collision_file = "controllers/supervisor/data/collisions.csv"
collisions = ld.load_file(collision_file)


# For each collision time, find the closest row in the trajectory data
collision_points = []

for t in collisions["time"]:
    closest_idx = (data["time"] - t).abs().idxmin()
    collision_points.append(data.loc[closest_idx])

collision_df = pd.DataFrame(collision_points)

print(collision_df.head())

total_collisions = len(collision_df.index)
total_time_sec = data["time"].iloc[-1]


import matplotlib.pyplot as plt
import numpy as np

# Create figure with reduced height
fig, ax = plt.subplots(figsize=(10, 4.7))

# Plot trajectory
ax.plot(data["x"], data["y"], label="Trajectory", color="blue", linewidth=2.5)

# Plot collisions if any
if len(collision_df.index):
    ax.scatter(collision_df["x"], collision_df["y"], color="red", marker='x', s=100, label="Collisions")

# Set labels and title with larger fonts
ax.set_xlabel("x position", fontsize=16)
ax.set_ylabel("y position", fontsize=16)
ax.set_title(
    f"Robot Trajectory with Collisions\nTotal Collisions: {total_collisions} | Total Time: {total_time_sec:.2f} seconds",
    fontsize=18
)
ax.tick_params(axis='both', labelsize=14)
ax.set_yticks(np.arange(0, 5, 1))  # y-ticks at step of 1

# Axis limits and small visual margins
ax.set_xlim(-0.7, 6.2)
ax.set_ylim(-0.5, 4.5)
ax.margins(x=0.033, y=0.02)

# Set aspect ratio so y is 2/3 the size of x (visually)
ax.set_aspect(1/1.7)  # x/y ratio

# Move legend: upper-left, 1.5 cm down
dpi = fig.dpi
offset_cm = 1.5
offset_in = offset_cm / 2.54
fig_height_in = fig.get_size_inches()[1]
y_offset = 1 - offset_in / fig_height_in

ax.legend(loc='upper left', bbox_to_anchor=(0, y_offset), fontsize=14)

# Grid and layout
ax.grid(True)
plt.tight_layout()
plt.show()

import pandas as pd
import matplotlib.pyplot as plt
import load_data as ld

# Load data
filename = "controllers/supervisor/data/ground_truth.csv"
data = ld.load_file(filename)

lights_file = "controllers/controller/data/Lights_detected.csv"
lights = pd.read_csv(lights_file)

# Clean column headers
lights.columns = lights.columns.str.strip()

# Plot
plt.figure(figsize=(10, 6))
plt.plot(data["x"], data["y"], label="Trajectory", color="blue")

# Plot lights by status
plt.scatter(lights[lights["status"] == 3]["x"], lights[lights["status"] == 3]["y"],
color='red', marker='o', s=100, label="Defective Light")

plt.scatter(lights[lights["status"] == 2]["x"], lights[lights["status"] == 2]["y"],
color='yellow', marker='o', s=100, label="Flickering Light")

plt.scatter(lights[lights["status"] == 1]["x"], lights[lights["status"] == 1]["y"],
color='green', marker='o', s=100, label="Nominal Light")

plt.xlabel("x position")
plt.ylabel("y position")
plt.title("Robot Trajectory with Lights")
plt.legend()
plt.grid(True)
plt.axis("equal")
plt.show()



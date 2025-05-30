# Implemented by: Linus
import load_data as ld
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy.signal import butter, filtfilt
from scipy.signal import TransferFunction, step



VERBOSE_plot_Data_T_in = False
VERBOSE_plot_Data_T_in_smooth = False
VERBOSE_plot_R_filtered = False
VERBOSE_dT_in_dt = False

filename = "controllers/controller/data/sensor_data.csv"
data = ld.load_file(filename)

print(data.head(50))


# Define a Butterworth low-pass filter
def lowpass_filter(signal, cutoff_freq, fs, order=3):
    nyquist = 0.5 * fs
    norm_cutoff = cutoff_freq / nyquist
    b, a = butter(order, norm_cutoff, btype='low', analog=False)
    return filtfilt(b, a, signal)



#transferfunction 

def stepresponse(s, C, R):
    Theta = 1/(C*s**2+s/R)
    return Theta

t = np.linspace(0, 1000, 500)


C = 0.1 #[J/K] thermal capacitance

QH = 0.1 #[W] power heater

results = []

for sensor_id, group in data.groupby('ID'):
    group = group.copy()
    group = group.sort_values('time')
    # Example usage:
    # Assuming time is in seconds and uniformly sampled
    fs = 1 / np.mean(np.diff(group['time']))  # Sampling frequency
    cutoff = 0.1  # Hz, adjust based on your data's dynamics
    

    group['T_in_filtered'] = lowpass_filter(group['T_in'], cutoff, fs)
    group['T_out_filtered'] = lowpass_filter(group['T_out'], cutoff, fs)
    


    # Compute dT_in/dt using smoothed T_in
    group['dT_in_dt'] = np.gradient(group['T_in_filtered'], group['time'])

    # Recalculate R using smoothed values
    numerator = group['T_in_filtered'] - group['T_out_filtered']
    denominator = QH - C * group['dT_in_dt']
    group['R_estimated'] = np.where(denominator != 0, numerator / denominator, np.nan)

    # Filter to reasonable range
    group['R_filtered'] = group['R_estimated']
    

    results.append(group)

# Combine and plot
df_result = pd.concat(results)

R = np.mean(df_result["R_filtered"])
print(f'R_mean with real data = {R}')


T_out_mean = np.mean(df_result["T_out_filtered"])
print(f'T_out_mean = {T_out_mean}')

T_star = 21.5

R_star = (T_star-T_out_mean)/QH

print(f"R* for T* = 21.5° = {R_star}")





###############################################################
#######################     stepresponse    #######################
###############################################################




# Measured R from your data
R_measured = R  # Replace this with your actual average R

# Define transfer functions for both
def create_tf(R):
    num = [1]
    den = [C, 1/R]
    return TransferFunction(num, den)

system_ideal = create_tf(R_star)
system_measured = create_tf(R_measured)

# Time vector for simulation
t = np.linspace(0, 1000, 500)  # seconds

# Step responses (response to constant QH)
t1, y1 = step(system_ideal, T=t)
t2, y2 = step(system_measured, T=t)

# Multiply by input magnitude (QH), and shift by T_out
T_ideal = QH * y1 + T_out_mean
T_measured = QH * y2 + T_out_mean

# Create figure
fig = plt.figure(figsize=(10, 3.5))
plt.plot(t1, T_ideal, label=f'Ideal R = {R_star:.1f}', lw = '2')
plt.plot(t2, T_measured, label=f'Measured R = {R_measured:.1f}', lw = '2')
plt.axhline(T_star, color='gray', linestyle='--', label='Desired Temp')

# Bigger font sizes
plt.title("Step Response of Greenhouse Temperature", fontsize=18)
plt.xlabel("Time [s]", fontsize=16)
plt.ylabel("Indoor Temperature T(t) [°C]", fontsize=16)
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)

dpi = fig.dpi
offset_cm = 4
offset_in = offset_cm / 2.54
fig_height_in = fig.get_size_inches()[1]
y_offset = 1 - offset_in / fig_height_in

plt.legend(loc='upper right', bbox_to_anchor=(1, y_offset), fontsize=14)

# Styling
plt.grid(True)
plt.tight_layout()
plt.show()



###############################################################
#######################     plotting    #######################
###############################################################


if VERBOSE_plot_R_filtered:
    # Plot combined R_filtered plot
    plt.figure(figsize=(10, 4))
    plt.plot(df_result["time"], df_result["R_filtered"])
    plt.title("Smoothed Thermal Resistance (All Sensors)")
    plt.xlabel("Time")
    plt.ylabel("R_filtered")
    plt.grid(True)
    plt.show()

if VERBOSE_plot_R_filtered:
    sensor_ids = df_result['ID'].unique()
    num_sensors = len(sensor_ids)

    fig, axes = plt.subplots(num_sensors, 1, figsize=(10, 3 * num_sensors), sharex=False)

    for i, sensor_id in enumerate(sensor_ids):
        ax = axes[i]
        sensor_data = df_result[df_result['ID'] == sensor_id]

        ax.plot(sensor_data["time"], sensor_data["R_filtered"])
        ax.set_title(f"Sensor ID {sensor_id} - R_filtered")
        ax.set_ylabel("R [K/W]")
        ax.set_xlim(sensor_data['time'].min(), sensor_data['time'].max())
        ax.grid(True)

    axes[-1].set_xlabel("Time (raw steps from data['time'])")
    plt.tight_layout()
    plt.show()

######## Data_T_in ########
if VERBOSE_plot_Data_T_in:

    sensor_ids = data['ID'].unique()
    num_sensors = len(sensor_ids)

    fig, axes = plt.subplots(num_sensors, 1, figsize=(10, 3 * num_sensors), sharex=False)

    for i, sensor_id in enumerate(sensor_ids):
        ax = axes[i]
        sensor_data = data[data['ID'] == sensor_id]

        ax.plot(sensor_data["time"].iloc[1:-1], sensor_data["T_in"].iloc[1:-1])
        ax.set_title(f"Sensor ID {sensor_id}")
        ax.set_ylabel("T_in [°C]")
        ax.set_xlim(sensor_data['time'].min(), sensor_data['time'].max())
        ax.grid(True)

    axes[-1].set_xlabel("Time (raw steps from data['time'])")
    plt.tight_layout()
    plt.show()


######## Data_T_in_smooth  ########

if VERBOSE_plot_Data_T_in_smooth:

    sensor_ids = df_result['ID'].unique()
    num_sensors = len(sensor_ids)

    fig, axes = plt.subplots(num_sensors, 1, figsize=(10, 3 * num_sensors), sharex=False)

    for i, sensor_id in enumerate(sensor_ids):
        ax = axes[i]
        sensor_data = df_result[df_result['ID'] == sensor_id]

        ax.plot(sensor_data["time"], sensor_data["T_in_filtered"], label="T_in_smooth", color="tab:blue")
        ax.set_title(f"Smoothed T_in for Sensor ID {sensor_id}")
        ax.set_ylabel("T_in_smooth [°C]")
        ax.set_xlim(sensor_data['time'].min(), sensor_data['time'].max())
        ax.grid(True)

    axes[-1].set_xlabel("Time")
    plt.tight_layout()
    plt.show()


if VERBOSE_dT_in_dt:



    sensor_ids = df_result['ID'].unique()
    num_sensors = len(sensor_ids)

    fig, axes = plt.subplots(num_sensors, 1, figsize=(10, 3 * num_sensors), sharex=False)

    for i, sensor_id in enumerate(sensor_ids):
        ax = axes[i]
        sensor_data = df_result[df_result['ID'] == sensor_id]

        ax.plot(sensor_data["time"], sensor_data["dT_in_dt"], label="dT_in_dt", color="tab:blue")
        ax.set_title(f"dT_in_dt for Sensor ID {sensor_id}")
        ax.set_ylabel("dT_in_dt")
        ax.set_xlim(sensor_data['time'].min(), sensor_data['time'].max())
        ax.grid(True)

    axes[-1].set_xlabel("Time")
    plt.tight_layout()
    plt.show()
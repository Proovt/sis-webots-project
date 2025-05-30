import pandas as pd
import matplotlib.pyplot as plt
import load_data as ld


# Load data
filename1 = "controllers/controller/data/ampVSt.csv"
data1 = pd.read_csv(filename1)


# Clean column headers
data1.columns = data1.columns.str.strip()


# Plot
plt.figure(figsize=(10, 6))


time = 1

plt.plot(data1.loc[data1["time"] == time, "frequency"], data1.loc[data1["time"] == time, "magnitude"])

plt.title("FFT")
plt.legend()
plt.grid(True)
#plt.axis("equal")
plt.show()
import matplotlib.pyplot as plt
import load_data as ld

# Load data
filename = "controllers/controller/data/light_data.csv"

data = ld.load_file(filename)

for id in data["ID"].unique():
    # Plot
    plt.figure(figsize=(8, 6))
    plt.plot(data.loc[data["ID"] == id, "frequency"], data.loc[data["ID"] == id, "magnitude"])

    plt.title("FFT")
    plt.legend()
    plt.grid(True)
    #plt.axis("equal")
    plt.show()
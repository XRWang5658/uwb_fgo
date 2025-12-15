import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Load the data
baseline = pd.read_csv("/home/xiangru/code/uwb_ws/src/uwb_fgo/data/baseline_path.csv")
optimized = pd.read_csv("/home/xiangru/code/uwb_ws/src/uwb_fgo/data/optimized_path.csv")

# Invert the Y-axis for the baseline data
baseline["y"] = -baseline["y"]

# Resample the data to align lengths
# Use numpy's linspace to interpolate both datasets to the same number of points
num_points = min(len(baseline), len(optimized))  # Use the smaller length for alignment
baseline_resampled = pd.DataFrame({
    "x": np.interp(np.linspace(0, len(baseline) - 1, num_points), np.arange(len(baseline)), baseline["x"]),
    "y": np.interp(np.linspace(0, len(baseline) - 1, num_points), np.arange(len(baseline)), baseline["y"]),
    "z": np.interp(np.linspace(0, len(baseline) - 1, num_points), np.arange(len(baseline)), baseline["z"]),
})
optimized_resampled = pd.DataFrame({
    "x": np.interp(np.linspace(0, len(optimized) - 1, num_points), np.arange(len(optimized)), optimized["x"]),
    "y": np.interp(np.linspace(0, len(optimized) - 1, num_points), np.arange(len(optimized)), optimized["y"]),
    "z": np.interp(np.linspace(0, len(optimized) - 1, num_points), np.arange(len(optimized)), optimized["z"]),
})

# Create the figure
plt.figure(figsize=(12, 8))

# X-axis comparison
plt.subplot(3, 1, 1)
plt.plot(baseline_resampled["x"], label="Baseline", alpha=0.7)
plt.plot(optimized_resampled["x"], label="Optimized", alpha=0.7)
plt.ylabel("X Position")
plt.title("X-Axis Comparison")
plt.legend()

# Y-axis comparison
plt.subplot(3, 1, 2)
plt.plot(baseline_resampled["y"], label="Baseline", alpha=0.7)
plt.plot(optimized_resampled["y"], label="Optimized", alpha=0.7)
plt.ylabel("Y Position")
plt.title("Y-Axis Comparison")
plt.legend()

# Z-axis comparison
plt.subplot(3, 1, 3)
plt.plot(baseline_resampled["z"], label="Baseline", alpha=0.7)
plt.plot(optimized_resampled["z"], label="Optimized", alpha=0.7)
plt.ylabel("Z Position")
plt.xlabel("Index")
plt.title("Z-Axis Comparison")
plt.legend()

# Show the plot
plt.tight_layout()
plt.show()
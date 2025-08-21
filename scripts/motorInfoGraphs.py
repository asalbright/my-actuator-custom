import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV file into a DataFrame
data = pd.read_csv("motor_data.csv")

# Extract columns
time = data["Time (s)"]
current = data["Current (A)"]
shaft_speed = data["Shaft Speed (rpm)"]
shaft_angle = data["Shaft Angle (deg)"]
position = data["Position (deg)"]

# Create subplots
fig, axs = plt.subplots(4, 1, figsize=(10, 16), sharex=True)

# Plot Current
axs[0].scatter(time, current, label="Current (A)", color="tab:blue", s=1)
axs[0].set_ylabel("Current (A)")
axs[0].set_title("Motor Current")
axs[0].grid(True, linestyle="--", alpha=0.5)

# Plot Shaft Speed
axs[1].scatter(time, shaft_speed, label="Shaft Speed (rpm)", color="tab:orange", s=1)
axs[1].set_ylabel("Shaft Speed (rpm)")
axs[1].set_title("Shaft Speed")
axs[1].grid(True, linestyle="--", alpha=0.5)

# Plot Shaft Angle
axs[2].scatter(time, shaft_angle, label="Shaft Angle (deg)", color="tab:green", s=1)
axs[2].set_ylabel("Shaft Angle (deg)")
axs[2].set_title("Shaft Angle")
axs[2].grid(True, linestyle="--", alpha=0.5)

# Plot Position
axs[3].scatter(time, position, label="Position (deg)", color="tab:red", s=1)
axs[3].set_ylabel("Position (deg)")
axs[3].set_title("Input Position (Sine Wave)")
axs[3].set_xlabel("Time (s)")
axs[3].grid(True, linestyle="--", alpha=0.5)

# Adjust layout with padding
fig.tight_layout(pad=3.0)

# Show the plot
plt.show()
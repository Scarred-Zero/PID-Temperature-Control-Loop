import pandas as pd
import matplotlib.pyplot as plt

# 1. Load the data
# Ensure 'run.csv' is in the same folder as this script
df = pd.read_csv("run.csv", encoding="utf-16")

# Convert timestamp from milliseconds to seconds for a cleaner X-axis
df["time_s"] = df["timestamp_ms"] / 1000.0

# 2. Create the figure and primary axis
fig, ax1 = plt.subplots(figsize=(10, 6))

# 3. Plot Temperature Data (Primary Y-Axis)
color_temp = "#D32F2F"  # A strong, professional red
ax1.set_xlabel("Time (Seconds)", fontweight="bold")
ax1.set_ylabel("Temperature (°C)", color=color_temp, fontweight="bold")

# Plot the measured temperature curve
ax1.plot(
    df["time_s"],
    df["measured_c"],
    color=color_temp,
    linewidth=2.5,
    label="Measured Temp (°C)",
)

# Plot the 60.0C Setpoint as a dashed line
ax1.plot(
    df["time_s"],
    df["setpoint_c"],
    color="black",
    linestyle="--",
    linewidth=1.5,
    label="Setpoint (60.0°C)",
)

# Add a subtle shaded region for your ±0.5°C validation band
ax1.fill_between(
    df["time_s"],
    df["setpoint_c"] - 0.5,
    df["setpoint_c"] + 0.5,
    color="gray",
    alpha=0.2,
    label="±0.5°C Steady-State Band",
)

ax1.tick_params(axis="y", labelcolor=color_temp)
ax1.set_ylim(20, 70)  # Pad the Y-axis slightly below ambient and above setpoint

# 4. Plot PID Output (Secondary Y-Axis)
ax2 = ax1.twinx()
color_pwm = "#1976D2"  # A strong, professional blue
ax2.set_ylabel("PID Output (PWM %)", color=color_pwm, fontweight="bold")

# Plot the control effort (PWM)
ax2.plot(
    df["time_s"],
    df["pid_output_pct"],
    color=color_pwm,
    linewidth=1.5,
    alpha=0.7,
    label="PID Control Effort (%)",
)

ax2.tick_params(axis="y", labelcolor=color_pwm)
ax2.set_ylim(0, 105)  # Cap PWM visually from 0 to 105%

# 5. Formatting and Legends
plt.title(
    "SIL Digital Twin: PID Thermal Step Response",
    fontsize=14,
    fontweight="bold",
    pad=15,
)

# Combine legends from both axes into one box
lines_1, labels_1 = ax1.get_legend_handles_labels()
lines_2, labels_2 = ax2.get_legend_handles_labels()
ax1.legend(lines_1 + lines_2, labels_1 + labels_2, loc="lower right", framealpha=0.9)

# Add a grid for readability
ax1.grid(True, linestyle=":", alpha=0.6)
fig.tight_layout()

# 6. Save and Display
plt.savefig("pid_step_response_hq.png", dpi=300)  # Saves a high-res image for LinkedIn
print("Plot saved as 'pid_step_response_hq.png'")
plt.show()

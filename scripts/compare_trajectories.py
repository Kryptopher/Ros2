#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import sys
import numpy as np

if len(sys.argv) < 2:
    print("Usage: python3 compare_trajectories.py <teensy_commands.csv>")
    sys.exit(1)

# Read Teensy commands
teensy_file = sys.argv[1]
teensy_df = pd.read_csv(teensy_file)

print(f"Loaded {len(teensy_df)} Teensy command points")

# Define the original velocity profile from action client
# {time, velocity} pairs
original_profile = [
    (0.0, 5.0),
    (5.0, 25.0),
    (10.0, 10.0),
    (15.0, 0.0),
    (20.0, 50.0),
    (25.0, 0.0),
    (26.0, 0.0)
]

# Create step-wise profile for plotting
profile_times = []
profile_velocities = []

for i in range(len(original_profile) - 1):
    t_start, v_start = original_profile[i]
    t_end, v_end = original_profile[i + 1]
    
    # Add points at segment boundaries
    profile_times.append(t_start)
    profile_velocities.append(v_start)
    profile_times.append(t_end - 0.001)  # Just before transition
    profile_velocities.append(v_start)

# Add final point
profile_times.append(original_profile[-1][0])
profile_velocities.append(original_profile[-1][1])

# Create plots
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(14, 10))
fig.suptitle('Action Client vs Teensy Commands Comparison', fontsize=16)

# Plot 1: Velocity comparison (rad/s)
ax1.plot(profile_times, profile_velocities, 'b-', 
         label='Action Client Input', linewidth=3, alpha=0.7)
ax1.plot(teensy_df['time'], teensy_df['velocity_rad_s'], 'r-', 
         label='Teensy Commands (rad/s)', linewidth=1, alpha=0.8)
ax1.set_ylabel('Velocity (rad/s)')
ax1.set_title('Velocity Profile: Input vs Commands')
ax1.set_xlim([0, 40])  # Add this line
ax1.legend()
ax1.grid(True, alpha=0.3)

# Plot 2: Steps per second sent to Teensy
ax2.plot(teensy_df['time'], teensy_df['steps_per_sec'], 'g-', 
         label='Steps/sec to Teensy', linewidth=1)
ax2.set_ylabel('Steps per Second')
ax2.set_title('Actual Teensy Commands (steps/sec)')
ax2.set_xlim([0, 40])  # Add this line
ax2.legend()
ax2.grid(True, alpha=0.3)

# Plot 3: Command rate (how often commands are sent)
time_diffs = np.diff(teensy_df['time'])
command_rates = 1.0 / time_diffs
ax3.plot(teensy_df['time'][1:], command_rates, 'm-', linewidth=0.5, alpha=0.7)
ax3.axhline(y=500, color='r', linestyle='--', label='Target: 500 Hz')
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Command Rate (Hz)')
ax3.set_title('Teensy Command Update Rate')
ax3.set_xlim([0, 40])  # Add this line
ax3.set_ylim([0, 600])
ax3.legend()
ax3.grid(True, alpha=0.3)
plt.tight_layout()

# Save plot
output_file = teensy_file.replace('.csv', '_comparison.png')
plt.savefig(output_file, dpi=150, bbox_inches='tight')
print(f"Comparison plot saved to: {output_file}")

# Print statistics
print(f"\nStatistics:")
print(f"  Command update rate: {command_rates.mean():.1f} Hz (avg), {command_rates.std():.1f} Hz (std)")
print(f"  Velocity range: {teensy_df['velocity_rad_s'].min():.2f} to {teensy_df['velocity_rad_s'].max():.2f} rad/s")
print(f"  Steps/sec range: {teensy_df['steps_per_sec'].min()} to {teensy_df['steps_per_sec'].max()}")

plt.show()

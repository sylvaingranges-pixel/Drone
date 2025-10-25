#!/usr/bin/env python3
"""
Example: Single scenario simulation
====================================

This script demonstrates how to use the drone control system for a single scenario.
"""

import numpy as np
import matplotlib.pyplot as plt
from drone_control import DroneLoadSystem, OptimalController, simulate_nonlinear, simulate_linear

# Define scenario
print("Setting up scenario...")
x0 = np.array([0.0, 0.0, 0.0, 0.0])  # Start at rest
x_target = 30.0  # Target position: 30m
Ts = 0.1  # Sampling time
N_horizon = 250  # Prediction horizon

# Create system and controller
system = DroneLoadSystem()
controller = OptimalController(system, Ts)

# Compute optimal trajectory
print(f"Computing optimal trajectory to move load from 0m to {x_target}m...")
u_opt, x_opt = controller.compute_trajectory(x0, x_target, N_horizon)

if u_opt is None:
    print("Failed to compute optimal trajectory!")
    exit(1)

print(f"Success! Control horizon: {N_horizon} steps ({N_horizon*Ts:.1f}s)")
print(f"Max control effort: {np.max(np.abs(u_opt)):.3f} m/s²")

# Simulate on linear model
print("Simulating on linear model...")
x_lin, x_load_lin, v_load_lin = simulate_linear(
    system, x0, u_opt.flatten(), controller.Ad, controller.Bd
)

# Simulate on non-linear model
print("Simulating on non-linear model with RK45...")
t_nonlin, x_nonlin, x_load_nonlin, v_load_nonlin = simulate_nonlinear(
    system, x0, u_opt.flatten(), Ts, N_horizon * Ts
)

# Results
print(f"\nFinal load position:")
print(f"  Target: {x_target:.2f}m")
print(f"  Linear model: {x_load_lin[-1]:.2f}m")
print(f"  Non-linear model: {x_load_nonlin[-1]:.2f}m")

# Create simple plot
fig, axes = plt.subplots(2, 2, figsize=(12, 8))
t = np.linspace(0, N_horizon * Ts, N_horizon + 1)

# Load position
axes[0, 0].plot(t, x_load_lin, 'b-', linewidth=2, label='Linear')
axes[0, 0].plot(t_nonlin, x_load_nonlin, 'r--', linewidth=2, label='Non-linear')
axes[0, 0].axhline(y=x_target, color='g', linestyle=':', linewidth=2, label='Target')
axes[0, 0].set_xlabel('Time (s)')
axes[0, 0].set_ylabel('Load Position (m)')
axes[0, 0].set_title('Load Position')
axes[0, 0].legend()
axes[0, 0].grid(True, alpha=0.3)

# Load velocity
axes[0, 1].plot(t, v_load_lin, 'b-', linewidth=2, label='Linear')
axes[0, 1].plot(t_nonlin, v_load_nonlin, 'r--', linewidth=2, label='Non-linear')
axes[0, 1].set_xlabel('Time (s)')
axes[0, 1].set_ylabel('Load Velocity (m/s)')
axes[0, 1].set_title('Load Velocity')
axes[0, 1].legend()
axes[0, 1].grid(True, alpha=0.3)

# Pendulum angle
axes[1, 0].plot(t, np.rad2deg(x_lin[2, :]), 'b-', linewidth=2, label='Linear')
axes[1, 0].plot(t_nonlin, np.rad2deg(x_nonlin[2, :]), 'r--', linewidth=2, label='Non-linear')
axes[1, 0].set_xlabel('Time (s)')
axes[1, 0].set_ylabel('Angle (deg)')
axes[1, 0].set_title('Pendulum Angle')
axes[1, 0].legend()
axes[1, 0].grid(True, alpha=0.3)

# Control input
t_u = np.linspace(0, t[-1], len(u_opt.flatten()))
axes[1, 1].step(t_u, u_opt.flatten(), 'b-', linewidth=2, where='post')
axes[1, 1].set_xlabel('Time (s)')
axes[1, 1].set_ylabel('Control Input (m/s²)')
axes[1, 1].set_title('Drone Acceleration')
axes[1, 1].grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('example_result.png', dpi=150, bbox_inches='tight')
print("\nPlot saved as: example_result.png")
plt.show()

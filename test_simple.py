#!/usr/bin/env python3
"""Quick test of non-linear optimization"""

import numpy as np
from drone_control import DroneLoadSystem, NonlinearOptimalController, simulate_nonlinear
import time

# Simple scenario: stopped start, 20m target
x0 = np.array([0.0, 0.0, 0.0, 0.0])
x_target = 20.0
Ts = 0.1
N_horizon = 150

system = DroneLoadSystem()
controller = NonlinearOptimalController(system, Ts)

print("Testing non-linear optimization...")
print(f"Initial state: {x0}")
print(f"Target: {x_target}m")
print(f"Horizon: {N_horizon} steps ({N_horizon*Ts}s)")

t_start = time.time()
u_opt, x_opt = controller.compute_trajectory(x0, x_target, N_horizon)
t_comp = time.time() - t_start

if u_opt is not None:
    print(f"\nOptimization successful in {t_comp:.2f}s!")
    print(f"Max control: {np.max(np.abs(u_opt)):.3f} m/s²")
    
    # Check final state
    x_load_final = x_opt[0, -1] + system.L * np.sin(x_opt[2, -1])
    v_load_final = x_opt[1, -1] + system.L * x_opt[3, -1] * np.cos(x_opt[2, -1])
    
    print(f"\nFinal state (from optimization):")
    print(f"  Load position: {x_load_final:.4f}m (target: {x_target}m)")
    print(f"  Load velocity: {v_load_final:.6f}m/s")
    print(f"  Angle: {np.rad2deg(x_opt[2, -1]):.4f}deg")
    print(f"  Angular velocity: {x_opt[3, -1]:.6f}rad/s")
    
    # Verify with RK45
    print("\nVerifying with RK45 simulation...")
    t_nonlin, x_nonlin, x_load_nonlin, v_load_nonlin = simulate_nonlinear(
        system, x0, u_opt.flatten(), Ts, N_horizon * Ts)
    
    print(f"Final state (RK45 verification):")
    print(f"  Load position: {x_load_nonlin[-1]:.4f}m")
    print(f"  Load velocity: {v_load_nonlin[-1]:.6f}m/s")
    print(f"  Angle: {np.rad2deg(x_nonlin[2, -1]):.4f}deg")
    
    # Check overshoot
    overshoot = np.max(x_load_nonlin) - x_target
    print(f"\nOvershoot: {overshoot:.4f}m ({100*overshoot/x_target:.2f}%)")
    
else:
    print("Optimization failed!")

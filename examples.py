"""
Example usage of the drone control system with different scenarios
"""

import numpy as np
from drone_control import (
    DroneWithSuspendedLoad, 
    OptimalController,
    simulate_linear,
    simulate_nonlinear,
    plot_results
)


def scenario_1_short_distance():
    """Scenario 1: Short distance transport"""
    print("\n" + "=" * 70)
    print("SCENARIO 1: Short Distance Transport (20m)")
    print("=" * 70)
    
    model = DroneWithSuspendedLoad()
    controller = OptimalController(model, N_horizon=100)
    
    # Initial state: small initial swing
    x0 = np.array([0.0, 0.0, 0.05, 0.0])
    
    # Target: 20m
    x_target = 20.0
    
    print(f"Transport load from 0m to {x_target}m")
    
    # Compute optimal control
    u_opt, x_opt = controller.compute_optimal_trajectory(
        x0, x_target, u_max=3.0, theta_max=np.pi/6
    )
    
    # Simulate
    x_lin, x_l_lin, v_l_lin = simulate_linear(model, u_opt, x0)
    t_nl, x_nl, x_l_nl, v_l_nl = simulate_nonlinear(model, u_opt, x0, dt=0.01)
    
    print(f"Final load position (linear): {x_l_lin[-1]:.2f} m")
    print(f"Final load position (nonlinear): {x_l_nl[-1]:.2f} m")
    print(f"Time: {t_nl[-1]:.2f} s")
    
    return model, u_opt, t_nl, x_nl, x_l_nl, v_l_nl, x_lin, x_l_lin, v_l_lin, x_target


def scenario_2_longer_distance():
    """Scenario 2: Longer distance transport"""
    print("\n" + "=" * 70)
    print("SCENARIO 2: Longer Distance Transport (100m)")
    print("=" * 70)
    
    model = DroneWithSuspendedLoad()
    controller = OptimalController(model, N_horizon=300)
    
    # Initial state
    x0 = np.array([0.0, 0.0, 0.02, 0.0])
    
    # Target: 100m
    x_target = 100.0
    
    print(f"Transport load from 0m to {x_target}m")
    
    # Compute optimal control
    u_opt, x_opt = controller.compute_optimal_trajectory(
        x0, x_target, u_max=3.0, theta_max=np.pi/6
    )
    
    # Simulate
    x_lin, x_l_lin, v_l_lin = simulate_linear(model, u_opt, x0)
    t_nl, x_nl, x_l_nl, v_l_nl = simulate_nonlinear(model, u_opt, x0, dt=0.01)
    
    print(f"Final load position (linear): {x_l_lin[-1]:.2f} m")
    print(f"Final load position (nonlinear): {x_l_nl[-1]:.2f} m")
    print(f"Time: {t_nl[-1]:.2f} s")
    
    return model, u_opt, t_nl, x_nl, x_l_nl, v_l_nl, x_lin, x_l_lin, v_l_lin, x_target


def scenario_3_large_initial_swing():
    """Scenario 3: Large initial swing angle"""
    print("\n" + "=" * 70)
    print("SCENARIO 3: Large Initial Swing (15 degrees)")
    print("=" * 70)
    
    model = DroneWithSuspendedLoad()
    controller = OptimalController(model, N_horizon=200)
    
    # Initial state: large swing
    x0 = np.array([0.0, 0.0, np.radians(15), 0.0])
    
    # Target: 50m
    x_target = 50.0
    
    print(f"Transport load from 0m to {x_target}m with initial angle {np.rad2deg(x0[2]):.1f}°")
    
    # Compute optimal control
    u_opt, x_opt = controller.compute_optimal_trajectory(
        x0, x_target, u_max=3.0, theta_max=np.pi/5
    )
    
    # Simulate
    x_lin, x_l_lin, v_l_lin = simulate_linear(model, u_opt, x0)
    t_nl, x_nl, x_l_nl, v_l_nl = simulate_nonlinear(model, u_opt, x0, dt=0.01)
    
    print(f"Final load position (linear): {x_l_lin[-1]:.2f} m")
    print(f"Final load position (nonlinear): {x_l_nl[-1]:.2f} m")
    print(f"Time: {t_nl[-1]:.2f} s")
    
    return model, u_opt, t_nl, x_nl, x_l_nl, v_l_nl, x_lin, x_l_lin, v_l_lin, x_target


def main():
    """Run multiple scenarios"""
    print("\n" + "=" * 70)
    print("Drone Control System - Multiple Scenarios")
    print("=" * 70)
    
    # Run scenario 1
    scenario_1_short_distance()
    
    # Run scenario 2
    scenario_2_longer_distance()
    
    # Run scenario 3
    model, u_opt, t_nl, x_nl, x_l_nl, v_l_nl, x_lin, x_l_lin, v_l_lin, x_target = scenario_3_large_initial_swing()
    
    # Plot the last scenario
    import matplotlib
    matplotlib.use('Agg')
    plot_results(t_nl, x_nl, x_l_nl, v_l_nl, x_lin, x_l_lin, v_l_lin, 
                u_opt, model, x_target)
    
    print("\n" + "=" * 70)
    print("All scenarios completed!")
    print("=" * 70)


if __name__ == "__main__":
    main()

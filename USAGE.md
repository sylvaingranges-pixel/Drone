# Drone Control System - Usage Guide

## Quick Start

### Installation

1. Clone the repository:
```bash
git clone https://github.com/sylvaingranges-pixel/Drone.git
cd Drone
```

2. Install dependencies:
```bash
pip install -r requirements.txt
```

### Running the System

#### Main Demonstration

Run the complete system demonstration:
```bash
python drone_control.py
```

This will:
- Display system parameters and models
- Compute optimal control trajectory
- Simulate both linear and nonlinear models
- Generate comprehensive plots in `results.png`

**Expected Output:**
- Transport time: ~20 seconds
- Load reaches 50m target position
- Linear model: <0.1m error, nearly zero velocity
- Nonlinear model: ~2m error due to air drag effects

#### Multiple Scenarios

Run different test scenarios:
```bash
python examples.py
```

This demonstrates:
1. Short distance transport (20m)
2. Long distance transport (100m)
3. Large initial swing angle (15°)

#### Verification Tests

Run the complete test suite:
```bash
python test_system.py
```

This validates:
- Module imports
- Model creation
- Linearization
- Discretization
- Controller initialization
- Optimization
- Simulation

All tests should pass (7/7).

## Understanding the Output

### Console Output

The system displays:

1. **System Parameters**: Physical properties of the drone-load system
2. **State Matrices**: Continuous and discrete-time linear models
3. **Optimization Results**: Control trajectory statistics
4. **Simulation Results**: Performance of linear and nonlinear models
5. **Summary**: Final comparison between models

### Generated Plots

The `results.png` file contains 8 subplots:

1. **Load Position**: Shows trajectory of the load
   - Blue line: Nonlinear model (with air drag)
   - Red dashed: Linear model (without drag)
   - Green dotted: Target position

2. **Load Velocity**: Shows how load velocity changes
   - Should approach zero at destination

3. **Drone Position**: Shows drone trajectory
   - Drone moves ahead to pull load, then slows down

4. **Drone Velocity**: Shows drone velocity profile
   - Should be zero at start and end

5. **Cable Angle**: Shows pendulum swing angle
   - Measured in degrees from vertical
   - Should be near zero at start and end

6. **Angular Velocity**: Shows rate of swing
   - Should dampen to zero

7. **Control Input**: Shows drone acceleration commands
   - Bounded by ±3 m/s²
   - Step function due to discrete-time control

8. **Phase Portrait**: Angle vs angular velocity
   - Shows system trajectory in phase space
   - Should spiral to origin (stable equilibrium)

## Customizing Parameters

### Modify System Parameters

Edit `drone_control.py` in the `DroneWithSuspendedLoad.__init__()` method:

```python
self.m_d = 40.0   # Drone mass (kg)
self.m_l = 24.0   # Load mass (kg)
self.L = 19.0     # Cable length (m)
self.Cd = 1.0     # Drag coefficient
self.S = 0.2      # Cross-sectional area (m²)
self.ts = 0.1     # Sampling time (s)
```

### Change Target Position

In the `main()` function:

```python
x_target_load = 50.0  # Change this value (meters)
```

### Adjust Control Constraints

In the controller call:

```python
u_opt, x_opt = controller.compute_optimal_trajectory(
    x0, 
    x_target_load,
    u_max=3.0,           # Max acceleration (m/s²)
    v_d_max=15.0,        # Max drone velocity (m/s)
    theta_max=np.pi/6,   # Max cable angle (radians)
    omega_max=0.5        # Max angular velocity (rad/s)
)
```

### Change Prediction Horizon

Longer horizons allow better planning but increase computation time:

```python
controller = OptimalController(model, N_horizon=200)  # Default: 200
```

For a T-second mission, use `N_horizon = T / ts`, e.g.:
- 10s mission: N_horizon = 100
- 20s mission: N_horizon = 200
- 30s mission: N_horizon = 300

### Modify Cost Function Weights

In `OptimalController.compute_optimal_trajectory()`:

```python
Q = np.diag([10.0, 1.0, 100.0, 10.0])  # Stage cost
R = np.array([[0.01]])                  # Control cost
Q_terminal = np.diag([500.0, 100.0, 500.0, 100.0])  # Terminal cost
```

Higher values = stronger penalty:
- Increase Q[2,2] to reduce swing more aggressively
- Decrease R to allow more aggressive control
- Increase Q_terminal for tighter terminal constraints

## Advanced Usage

### Using the Model Programmatically

```python
from drone_control import DroneWithSuspendedLoad, OptimalController
import numpy as np

# Create model
model = DroneWithSuspendedLoad()

# Get linearized model
A, B = model.linearize()
print("Continuous-time model:")
print(f"A = \n{A}")
print(f"B = \n{B}")

# Get discrete-time model
Ad, Bd = model.discretize(A, B)
print("\nDiscrete-time model:")
print(f"Ad = \n{Ad}")
print(f"Bd = \n{Bd}")

# Create controller
controller = OptimalController(model, N_horizon=150)

# Define initial state and target
x0 = np.array([0.0, 0.0, 0.05, 0.0])  # [x_d, v_d, theta, omega]
x_target = 30.0  # meters

# Compute optimal control
u_opt, x_opt = controller.compute_optimal_trajectory(x0, x_target)

print(f"\nOptimal trajectory computed:")
print(f"Control sequence shape: {u_opt.shape}")
print(f"State trajectory shape: {x_opt.shape}")
```

### Custom Simulation

```python
from drone_control import simulate_linear, simulate_nonlinear

# Simulate linear model
x_lin, x_l_lin, v_l_lin = simulate_linear(model, u_opt, x0)

# Simulate nonlinear model with custom time step
t_nl, x_nl, x_l_nl, v_l_nl = simulate_nonlinear(
    model, u_opt, x0, dt=0.005  # Smaller step for higher accuracy
)

print(f"Linear final position: {x_l_lin[-1]:.2f} m")
print(f"Nonlinear final position: {x_l_nl[-1]:.2f} m")
```

### Custom Visualization

```python
from drone_control import plot_results

# Generate plots
plot_results(
    t_nl, x_nl, x_l_nl, v_l_nl,     # Nonlinear results
    x_lin, x_l_lin, v_l_lin,         # Linear results  
    u_opt,                            # Control input
    model,                            # Model instance
    x_target                          # Target position
)
```

## Troubleshooting

### Optimization Infeasible

**Problem**: "Optimization status: infeasible"

**Solutions**:
1. Increase prediction horizon: `N_horizon = 200` or more
2. Relax constraints: increase `u_max`, `theta_max`, `omega_max`
3. Choose closer target or start with smaller initial angle
4. Increase maximum time: longer horizon = more time to reach target

### Optimization Inaccurate

**Problem**: "Solution may be inaccurate"

**Solutions**:
1. Try different solver: Switch between ECOS and OSQP
2. Adjust solver settings: `problem.solve(solver=cp.OSQP, max_iter=10000)`
3. Scale the problem: Normalize state and input variables
4. Check if problem is well-conditioned

### Large Difference Between Linear and Nonlinear

**Problem**: Nonlinear model deviates significantly from linear

**Explanation**: This is expected! The differences show:
- Air drag effects (quadratic in velocity)
- Nonlinear pendulum dynamics at larger angles
- Model mismatch

**To Reduce**:
1. Use smaller velocities (reduce `u_max`)
2. Keep cable angle small (reduce `theta_max`)
3. Use longer settling time (increase `N_horizon`)
4. Consider implementing nonlinear MPC

### Import Errors

**Problem**: Cannot import modules

**Solutions**:
1. Ensure all dependencies installed: `pip install -r requirements.txt`
2. Check Python version: Requires Python 3.8+
3. Install missing packages individually: `pip install numpy scipy matplotlib cvxpy`

## Performance Notes

- **Optimization time**: ~0.5-2 seconds depending on horizon
- **Simulation time**: ~1-5 seconds depending on time span
- **Memory usage**: <500 MB for typical scenarios
- **Recommended horizon**: 100-300 steps (10-30 seconds)

## Next Steps

1. **Experiment**: Try different initial conditions and targets
2. **Analyze**: Study the phase portraits and trajectories
3. **Extend**: Modify for 2D or 3D motion
4. **Improve**: Implement nonlinear MPC or robust control
5. **Compare**: Test different cost function weights

For detailed mathematical derivations, see `TECHNICAL.md`.

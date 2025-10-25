# Drone Control System with Suspended Load

This project implements an optimal control system for a drone (40kg) carrying a suspended load (24kg) at 19m length using Model Predictive Control (MPC).

## System Description

### Physical Parameters
- **Drone mass**: 40 kg
- **Load mass**: 24 kg  
- **Cable length**: 19 m
- **Load surface area**: 0.2 m²
- **Drag coefficient**: 1.0
- **Air density**: 1.225 kg/m³

### State Variables
The system state is represented by 4 variables:
- `x_d`: Drone position in X-axis (m)
- `v_d`: Drone velocity (m/s)
- `theta`: Pendulum angle from vertical (rad)
- `omega`: Pendulum angular velocity (rad/s)

### Control Input
- `u`: Drone acceleration (m/s²)

### Output Variables
- `x_load`: Load position (m)
- `v_load`: Load velocity (m/s)

## Models

### 1. Non-linear Model
The non-linear model includes:
- Complete pendulum dynamics
- Aerodynamic drag on the load: `F_drag = -0.5 * ρ * Cd * S * v_load * |v_load|`
- Coupled drone-load dynamics

### 2. Linearized Model
Linearized around equilibrium point:
- Load directly below drone (theta = 0)
- Zero velocities
- No aerodynamic effects
- Small angle approximation

State-space representation:
```
dx/dt = A*x + B*u
y = C*x
```

### 3. Discrete-time Model
- Sampling time: Ts = 0.1s
- Obtained via matrix exponential method
- Used for optimal control computation

## Optimal Controller

The controller uses convex optimization (CVXPY with OSQP solver) to compute trajectories that:

1. **Move the load to target position**: Minimize position error
2. **Eliminate oscillations**: Terminal constraints ensure zero velocity and zero angle
3. **Minimize control effort**: Balance between performance and efficiency
4. **Respect constraints**: 
   - Maximum acceleration limits
   - Angle safety limits (±45°)

### Objective Function
The cost function combines:
- Running cost: penalizes deviation from target and control effort
- Terminal cost: strong penalty on final state error

## Validation

The optimal control is tested on:
1. **Linear model**: Discrete-time simulation
2. **Non-linear model**: RK45 ODE solver integration

This validates that the controller works on the realistic non-linear system with aerodynamic drag.

## Test Scenarios

The system is tested with various scenarios:

1. **Stopped start → 20m**: Starting from rest, move 20m
2. **Stopped start → 40m**: Starting from rest, move 40m
3. **Stopped start → 80m**: Starting from rest, move 80m
4. **Moving start → 20m**: Initial velocity 2 m/s, small angle
5. **Moving start → 40m**: Initial velocity 1.5 m/s with angle and angular velocity

## Installation

```bash
pip install -r requirements.txt
```

### Dependencies
- numpy: Numerical computations
- scipy: ODE solvers and linear algebra
- matplotlib: Visualization
- cvxpy: Convex optimization

## Usage

### Run All Test Scenarios

```bash
python drone_control.py
```

This will:
1. Display system parameters and models
2. Run all test scenarios
3. Generate plots for each scenario showing:
   - Optimal trajectory (position, velocity, angle)
   - Control input
   - Linear model response
   - Non-linear model response
   - Comparison between models

### Run a Single Scenario

```bash
python example.py
```

This simpler example demonstrates how to:
- Set up a custom scenario
- Compute optimal trajectory
- Simulate on both linear and non-linear models
- Generate a basic comparison plot

## Output

For each scenario, the program generates:
- Console output with initial/final states and performance metrics
- **Optimization time**: Time taken to compute the optimal trajectory (1-4 seconds)
- PNG plot file with comprehensive visualization

After all scenarios, a performance summary table shows:
- Optimization time for each scenario
- Control horizon length
- Average optimization time

The fast optimization times (1-4 seconds) make this approach suitable for closed-loop Model Predictive Control with re-planning.

### Plot Structure
Each plot contains 9 subplots organized in 3 rows:
1. **Optimal Trajectory**: Position, velocity, and angle from optimization
2. **Linear Model**: Simulated response on linearized model
3. **Non-linear Model**: Simulated response on full non-linear model with comparison

## Results

The controller successfully:
- ✅ Moves the load to target positions (20m, 40m, 80m) on the linearized model
- ✅ Eliminates oscillations at arrival (zero angle, zero velocity) on the linearized model
- ✅ Handles both stopped and moving initial conditions
- ✅ **Fast optimization**: 1-4 seconds per trajectory, suitable for real-time MPC
- ✅ Respects physical constraints

**Important Note on Non-linear Model Behavior:**
The controller is designed using the linearized model (as required by the problem statement). When tested on the full non-linear model with aerodynamic drag, there are significant deviations from the planned trajectory, especially for:
- Longer distances (40m, 80m)
- Higher velocities where aerodynamic drag becomes significant
- Large angles where the small-angle approximation breaks down

This is **expected behavior** and demonstrates the limitations of linearized control approaches. The comparison between linear and non-linear responses is educational and shows where linearization is valid (small angles, short distances) and where it breaks down (large angles, longer distances, aerodynamic effects).

For better performance on the non-linear system, advanced techniques would be needed:
- Iterative linearization with MPC re-planning
- Direct non-linear trajectory optimization
- Robust control with uncertainty bounds

## Mathematical Details

### Equations of Motion

**Drone dynamics:**
```
ẋ_d = v_d
v̇_d = u
```

**Pendulum dynamics (non-linear):**
```
θ̇ = ω
ω̇ = (-m_l * g * sin(θ) - F_drag * cos(θ) - m_l * u * cos(θ)) / (m_l * L)
```

**Aerodynamic drag:**
```
F_drag = -0.5 * ρ * Cd * S * v_load * |v_load|
```

**Load position:**
```
x_load = x_d + L * sin(θ)
v_load = v_d + L * ω * cos(θ)
```

### Linearization

At equilibrium (θ = 0, ω = 0, v_d = 0):

```
A = [0    1    0      0   ]
    [0    0    0      0   ]
    [0    0    0      1   ]
    [0    0  -g/L    0   ]

B = [  0  ]
    [  1  ]
    [  0  ]
    [-1/L ]

C = [1  0   L   0]
    [0  1   0   L]
```

## License

MIT License

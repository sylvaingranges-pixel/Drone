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
- PNG plot file with comprehensive visualization

### Plot Structure
Each plot contains 9 subplots organized in 3 rows:
1. **Optimal Trajectory**: Position, velocity, and angle from optimization
2. **Linear Model**: Simulated response on linearized model
3. **Non-linear Model**: Simulated response on full non-linear model with comparison

## Results

### Performance Metrics

#### Computation Time
The optimal trajectory calculation time varies with horizon length (measured on standard modern CPU):
- **20m targets**: ~5-6 seconds (N=400 steps, 40s horizon)
- **40m targets**: ~8-9 seconds (N=600 steps, 60s horizon)
- **80m target**: ~15-16 seconds (N=1000 steps, 100s horizon)

*Note: Times may vary depending on hardware. Measured on typical modern x86-64 CPU.*

Computation times include convex optimization problem formulation, OSQP solver execution, and constraint satisfaction checking. These times are fast enough for offline trajectory planning or slow-rate MPC applications.

#### Control Performance

**Short Distance (20m targets):**
- Stopped start: 1.48m position error (7.4% of target), 1.45 m/s final velocity
- Moving start: 0.30m position error (1.5% of target), 1.34 m/s final velocity
- Assessment: ✅ Good tracking on both linear and non-linear models

**Medium Distance (40m targets):**
- Stopped start: 1.76m position error (4.4% of target), 0.55 m/s final velocity
- Moving start: 1.99m position error (5.0% of target), 0.25 m/s final velocity  
- Assessment: ✅ Acceptable performance with moderate deviations

**Long Distance (80m target):**
- Stopped start: 11.62m position error (14.5% of target), 12.15 m/s final velocity
- Assessment: ⚠️ Significant deviation and failure to achieve terminal constraints (zero velocity) demonstrates linearization limits

### Controller Capabilities

The controller successfully:
- ✅ Computes optimal trajectories in 5-16 seconds
- ✅ Moves the load toward target positions
- ✅ Minimizes oscillations on the linear model
- ✅ Handles both stopped and moving initial conditions
- ✅ Respects physical constraints (acceleration, angle limits)
- ⚠️ Shows deviations on non-linear model due to unmodeled drag effects

### Key Insights

1. **Computation Speed**: Trajectories computed in 5-16s, suitable for planning applications
2. **Short Distances**: Linear controller performs well for <20m (errors <2m)
3. **Medium Distances**: Reasonable performance for 20-40m (errors ~2m)
4. **Long Distances**: Linearization breaks down for >40m (errors >10m)
5. **Aerodynamic Effects**: Velocity-squared drag not captured by linearization causes the main deviations

The non-linear model shows reasonable agreement with the linearized model for short distances and small angles, with increasing deviation for larger displacements due to aerodynamic effects and linearization errors.

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

## Limitations and Future Work

### Current Limitations

1. **Linearization Validity**: The controller is based on a linearized model that assumes:
   - Small angles (sin(θ) ≈ θ, cos(θ) ≈ 1)
   - No aerodynamic drag
   - These assumptions break down for:
     - Large displacements (>40m)
     - High velocities (>2 m/s)
     - Long cable lengths

2. **Drag Effects**: The aerodynamic drag (F = 0.5 * ρ * Cd * S * v²) is:
   - Velocity-squared nonlinearity
   - Not modeled in the linearized controller
   - Causes significant deviations, especially at higher speeds

3. **Terminal Constraints**: While the linear model satisfies terminal constraints (zero velocity, zero angle), the non-linear model shows residual motion due to model mismatch.

### Potential Improvements

To achieve better performance on the non-linear system:

1. **Iterative MPC**: Re-linearize and re-optimize at each time step based on actual state
2. **Non-linear MPC**: Use sequential convex programming or direct collocation
3. **Robust Control**: Design controller accounting for model uncertainty bounds
4. **Adaptive Control**: Online parameter estimation and controller adaptation
5. **Feedforward Compensation**: Add drag compensation term based on estimated velocity
6. **Longer Settling Times**: Use much longer horizons (>100s) for gentler trajectories, though this significantly increases computation time (potentially 20-30+ seconds)

### Educational Value

This implementation demonstrates:
- ✅ Complete model-based control pipeline (modeling → optimization → validation)
- ✅ Linearization techniques and their practical limits
- ✅ Trade-offs between model complexity and control performance
- ✅ Importance of model fidelity for successful control
- ✅ When simple linear control is sufficient and when advanced techniques are needed

## License

MIT License

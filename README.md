# Drone with Suspended Load Control System

This project implements an optimal control system for a drone carrying a suspended load. The system is designed to transport the load to a target position as quickly as possible while ensuring zero oscillation at the destination.

## System Description

### Physical Parameters
- **Drone mass**: 40 kg
- **Load mass**: 24 kg  
- **Cable length**: 19 m
- **Air drag coefficient**: 1.0
- **Load cross-sectional area**: 0.2 m²
- **Air density**: 1.225 kg/m³

### Model

The system is modeled in 1D (X-axis) with the following components:

#### 1. Nonlinear Model
The nonlinear dynamics include:
- Drone position and velocity
- Cable angle and angular velocity
- Air drag on the load: F_drag = -0.5 * ρ * Cd * S * v_l * |v_l|
- Pendulum dynamics with moving support

**State variables**: [x_d, v_d, θ, ω]
- x_d: drone position (m)
- v_d: drone velocity (m/s)
- θ: cable angle from vertical (rad)
- ω: angular velocity (rad/s)

**Input**: a_d (drone acceleration in m/s²)

**Outputs**: 
- x_l: load position (m)
- v_l: load velocity (m/s)

#### 2. Linearized Model
Linearization around equilibrium point:
- θ = 0 (load directly below drone)
- ω = 0 (no swing)
- v_d = 0 (no velocity)
- No aerodynamic forces

The linearized system matrices A and B are derived from the nonlinear equations.

#### 3. Discrete-Time Model
The continuous-time linear model is discretized using zero-order hold with sampling time ts = 0.1s.

### Optimal Controller

The controller uses Model Predictive Control (MPC) with CVXPY to solve a constrained optimization problem:

**Objective**: Minimize transport time while ensuring:
- Load reaches target position
- Load velocity = 0 at destination
- Cable angle = 0 at destination (no oscillation)
- Drone velocity = 0 at destination

**Constraints**:
- Maximum drone acceleration
- Maximum cable angle
- Maximum angular velocity
- Terminal conditions for zero oscillation

## Installation

Install required dependencies:

```bash
pip install -r requirements.txt
```

## Usage

Run the main control system:

```bash
python drone_control.py
```

This will:
1. Display system parameters
2. Compute the linearized and discrete-time models
3. Solve the optimal control problem
4. Simulate the system on both linear and nonlinear models
5. Generate comparison plots

## Output

The program generates:
- Console output with detailed system information
- `results.png`: Comprehensive plots showing:
  - Load position and velocity
  - Drone position and velocity  
  - Cable angle and angular velocity
  - Control input (drone acceleration)
  - Phase portrait
  - Comparison between linear and nonlinear models

## Results

The optimal controller successfully:
- Transports the load to the target position
- Minimizes transport time
- Ensures zero oscillation at destination
- Brings both load and drone to complete stop

The nonlinear simulation uses RK45 (Runge-Kutta) ODE solver to accurately capture the system dynamics including air drag effects.

## Technical Details

### Numerical Methods
- **ODE Solver**: RK45 (Runge-Kutta-Fehlson) with adaptive step size
- **Optimization**: CVXPY with ECOS solver
- **Discretization**: Zero-order hold (ZOH)

### Model Validation
The implementation includes:
- Comparison between linear and nonlinear models
- Verification of terminal constraints
- Phase portrait analysis

## References

This implementation is based on classical pendulum dynamics and optimal control theory, adapted for a drone with suspended load system.

# Technical Documentation: Drone with Suspended Load Control System

## 1. System Description

This system models a drone carrying a suspended load via a cable. The control objective is to transport the load from an initial position to a target position as quickly as possible while ensuring zero oscillation at the destination.

### Physical Parameters
- **Drone mass (m_d)**: 40 kg
- **Load mass (m_l)**: 24 kg  
- **Cable length (L)**: 19 m
- **Air drag coefficient (Cd)**: 1.0
- **Load cross-sectional area (S)**: 0.2 m²
- **Air density (ρ)**: 1.225 kg/m³
- **Gravity (g)**: 9.81 m/s²

## 2. Nonlinear Model

### State Variables
The system state is defined by:
- **x_d**: drone position in X direction (m)
- **v_d**: drone velocity in X direction (m/s)
- **θ**: cable angle from vertical (rad)
- **ω**: angular velocity of cable (rad/s)

State vector: **x** = [x_d, v_d, θ, ω]ᵀ

### Input
- **a_d**: drone acceleration in X direction (m/s²)

### Output Variables
- **x_l**: load position = x_d + L·sin(θ)
- **v_l**: load velocity = v_d + L·ω·cos(θ)

### Nonlinear Dynamics

The system dynamics are derived from the equations of motion for a pendulum with a moving support point:

```
dx_d/dt = v_d

dv_d/dt = a_d  (control input)

dθ/dt = ω

dω/dt = -(g/L)·sin(θ) - (a_d/L)·cos(θ) + (F_drag/(m_l·L))·cos(θ)
```

where the air drag force on the load is:

```
F_drag = -0.5·ρ·Cd·S·v_l·|v_l|
```

This is a quadratic drag model that depends on the square of velocity.

### Physical Interpretation

1. The term **-(g/L)·sin(θ)** represents the restoring force of gravity acting on the pendulum
2. The term **-(a_d/L)·cos(θ)** represents the effect of drone acceleration on the pendulum
3. The term **F_drag/(m_l·L)·cos(θ)** represents the effect of air resistance on the load

## 3. Linearized Model

### Linearization Point (Equilibrium)
The system is linearized around the equilibrium point where:
- θ = 0 (load directly below drone)
- ω = 0 (no swing)
- v_d = 0 (no velocity)
- F_drag = 0 (no aerodynamic forces)

### Small Angle Approximations
- sin(θ) ≈ θ
- cos(θ) ≈ 1
- v_l ≈ v_d + L·ω

At equilibrium with v_l = 0, the drag force is zero.

### Linearized State-Space Model

The linearized continuous-time system is:

```
ẋ = A·x + B·u
y = C·x + D·u
```

where:

**State matrix A:**
```
     ┌                        ┐
     │  0    1    0      0    │
A =  │  0    0    0      0    │
     │  0    0    0      1    │
     │  0    0  -g/L    0    │
     └                        ┘
```

With L=19m and g=9.81 m/s²:
```
     ┌                              ┐
     │  0    1       0        0     │
A =  │  0    0       0        0     │
     │  0    0       0        1     │
     │  0    0    -0.5163     0     │
     └                              ┘
```

**Input matrix B:**
```
     ┌        ┐
     │   0    │
B =  │   1    │
     │   0    │
     │ -1/L   │
     └        ┘
```

With L=19m:
```
     ┌          ┐
     │    0     │
B =  │    1     │
     │    0     │
     │ -0.0526  │
     └          ┘
```

**Output matrix C:**
```
     ┌                    ┐
     │  1   0   L   0    │  (load position)
C =  │  0   1   0   L    │  (load velocity)
     └                    ┘
```

## 4. Discrete-Time Model

The continuous-time linear system is discretized using zero-order hold (ZOH) with sampling time **ts = 0.1 s**.

### Discretization Formulas

```
Ad = e^(A·ts)

Bd = ∫[0,ts] e^(A·τ) dτ · B
```

This is computed using matrix exponential:

```
     ┌          ┐
     │  A·ts  B·ts │
M =  │             │
     │   0      0  │
     └          ┘

exp(M) = ┌          ┐
         │  Ad   Bd  │
         │   0    I  │
         └          ┘
```

### Resulting Discrete Matrices

**Discrete state matrix Ad:**
```
      ┌                                    ┐
      │  1.0000   0.1000     0       0     │
Ad =  │  0        1.0000     0       0     │
      │  0        0       0.9974   0.0999  │
      │  0        0      -0.0516   0.9974  │
      └                                    ┘
```

**Discrete input matrix Bd:**
```
      ┌          ┐
      │  0.0050  │
Bd =  │  0.1000  │
      │ -0.0003  │
      │ -0.0053  │
      └          ┘
```

## 5. Optimal Control Problem

### Objective
Minimize the cost function:

```
J = Σ[k=0 to N-1] [(xₖ - x_target)ᵀ·Q·(xₖ - x_target) + uₖᵀ·R·uₖ] 
    + (xₙ - x_target)ᵀ·Q_terminal·(xₙ - x_target)
```

### Cost Matrices

**Stage cost on state (Q):**
```
Q = diag([10, 1, 100, 10])
```
- High penalty on angle (100) to minimize swing
- Moderate penalty on position (10)
- Lower penalty on velocities (1, 10)

**Control cost (R):**
```
R = 0.01
```
- Low penalty to allow aggressive control

**Terminal cost (Q_terminal):**
```
Q_terminal = diag([500, 100, 500, 100])
```
- Very high penalty to enforce zero oscillation at destination

### Constraints

**State Constraints:**
- |x_d| ≤ 100 m (drone position)
- |v_d| ≤ 15 m/s (drone velocity)
- |θ| ≤ π/6 rad ≈ 30° (cable angle)
- |ω| ≤ 0.5 rad/s (angular velocity)

**Input Constraints:**
- |a_d| ≤ 3 m/s² (maximum acceleration)

**Terminal Constraints (at time N):**
- |θₙ| ≤ 0.005 rad ≈ 0.3° (near zero angle)
- |ωₙ| ≤ 0.005 rad/s (near zero angular velocity)
- |v_d,ₙ| ≤ 0.005 m/s (near zero drone velocity)
- |x_l,ₙ - x_target| ≤ 0.05 m (load at target)

### Solution Method

The optimization problem is formulated as a convex quadratic program and solved using:
- Primary solver: ECOS (Embedded Conic Solver)
- Fallback solver: OSQP (Operator Splitting Quadratic Program)

Both solvers are efficient for real-time MPC applications.

## 6. Simulation

### Linear Model Simulation
The discrete-time linear model is simulated by iterating:
```
xₖ₊₁ = Ad·xₖ + Bd·uₖ
```

### Nonlinear Model Simulation
The nonlinear model is simulated using:
- **Method**: RK45 (Runge-Kutta-Fehlson)
- **Tolerance**: rtol=1e-6, atol=1e-9
- **Step size**: Adaptive (controlled by solver)
- **Evaluation step**: dt=0.01s for output

The RK45 method is a 4th/5th order adaptive Runge-Kutta method that provides accurate solutions for stiff and non-stiff ODEs.

## 7. Results and Analysis

### Expected Behavior

1. **Linear Model**: The controller is optimized for the linear model, so it performs very well with minimal error in reaching the target.

2. **Nonlinear Model**: Shows deviations due to:
   - Air drag effects (quadratic in velocity)
   - Nonlinear pendulum dynamics at larger angles
   - Coupling between horizontal motion and drag

3. **Key Observations**:
   - The difference between linear and nonlinear responses demonstrates model mismatch
   - Air drag acts as a damping force on the load
   - At higher velocities, drag becomes significant
   - The controller successfully dampens oscillations in both models

### Performance Metrics

For a typical 50m transport:
- **Time horizon**: 15-20 seconds
- **Maximum control**: 3 m/s²
- **Final position error**: < 0.1 m (linear), < 2 m (nonlinear)
- **Final velocity**: < 0.01 m/s (linear), < 1 m/s (nonlinear)
- **Final angle**: < 0.5° (linear), < 6° (nonlinear)

## 8. Implementation Notes

### Numerical Considerations

1. **Matrix Exponential**: Computed using `scipy.linalg.expm` with high precision
2. **ODE Integration**: RK45 with adaptive stepping ensures accuracy
3. **Optimization**: Convex formulation guarantees global optimum

### Computational Complexity

- **Optimization**: O(N·n²) where N is horizon length, n is state dimension
- **Linear simulation**: O(N) 
- **Nonlinear simulation**: O(N·M) where M is number of integration steps

### Extensions

Possible enhancements:
1. **2D/3D model**: Extend to full 3D dynamics
2. **Wind disturbances**: Add external forces
3. **MPC with replanning**: Recompute control at each step
4. **Nonlinear MPC**: Optimize directly on nonlinear model
5. **Robust control**: Account for uncertainties and disturbances

## 9. References

- Fliess, M., et al. "Flatness and defect of non-linear systems." International journal of control (1995)
- Boyd, S., & Vandenberghe, L. "Convex optimization." Cambridge university press (2004)
- Mayne, D. Q., et al. "Constrained model predictive control." Automatica (2000)

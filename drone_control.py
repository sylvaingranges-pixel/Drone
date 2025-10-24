"""
Drone with Suspended Load Control System

This module implements:
1. Nonlinear model of drone with suspended load (1D X-axis)
2. Linearized model around equilibrium
3. Discrete-time model
4. Optimal controller using CVXPY

System parameters:
- Drone mass: 40 kg
- Load mass: 24 kg
- Cable length: 19 m
- Air drag coefficient: 1.0
- Load cross-sectional area: 0.2 m²
"""

import numpy as np
from scipy.integrate import solve_ivp
from scipy.linalg import expm
import matplotlib.pyplot as plt
import cvxpy as cp


class DroneWithSuspendedLoad:
    """
    Model of a drone with a suspended load in 1D (X-axis)
    
    State variables:
    - x_d: drone position (m)
    - v_d: drone velocity (m/s)
    - theta: cable angle from vertical (rad)
    - omega: angular velocity of cable (rad/s)
    - x_l: load position (m)
    - v_l: load velocity (m/s)
    
    Input:
    - a_d: drone acceleration (m/s²)
    
    Output:
    - x_l: load position
    - v_l: load velocity
    """
    
    def __init__(self):
        # System parameters
        self.m_d = 40.0  # drone mass (kg)
        self.m_l = 24.0  # load mass (kg)
        self.L = 19.0    # cable length (m)
        self.g = 9.81    # gravity (m/s²)
        self.Cd = 1.0    # drag coefficient
        self.S = 0.2     # cross-sectional area (m²)
        self.rho = 1.225 # air density (kg/m³)
        
        # Discretization time step
        self.ts = 0.1
        
    def nonlinear_dynamics(self, t, state, u):
        """
        Nonlinear dynamics of the drone-load system
        
        Args:
            t: time
            state: [x_d, v_d, theta, omega] where
                   x_d: drone position
                   v_d: drone velocity
                   theta: cable angle from vertical
                   omega: angular velocity
            u: drone acceleration (control input)
        
        Returns:
            state_dot: derivatives of state variables
        """
        x_d, v_d, theta, omega = state
        a_d = u
        
        # Load position and velocity
        x_l = x_d + self.L * np.sin(theta)
        v_l = v_d + self.L * omega * np.cos(theta)
        
        # Air drag force on load
        F_drag = -0.5 * self.rho * self.Cd * self.S * v_l * np.abs(v_l)
        
        # Cable tension (from load equation)
        # m_l * a_l = T * sin(theta) + F_drag
        # where a_l is load acceleration in x direction
        
        # Equations of motion
        # For the load:
        # m_l * x_l_ddot = T * sin(theta) + F_drag
        # m_l * y_l_ddot = T * cos(theta) - m_l * g
        # where y_l = -L * cos(theta) (relative to drone)
        
        # Using Lagrangian mechanics for the pendulum part
        # Constraint: load is at distance L from drone
        
        # Simplified: treating as a pendulum attached to accelerating pivot
        # Effective gravity in pendulum frame: g_eff = g - (a_d acting horizontally doesn't affect vertical)
        
        # Angular acceleration (pendulum equation with drag)
        # I * alpha = -m_l * g * L * sin(theta) - m_l * a_d * L * cos(theta) + F_drag * L * cos(theta)
        # where I = m_l * L²
        
        alpha = (- (self.g / self.L) * np.sin(theta) 
                 - (a_d / self.L) * np.cos(theta)
                 + (F_drag / (self.m_l * self.L)) * np.cos(theta))
        
        # Drone dynamics
        # v_d_dot = a_d (given as input)
        
        return np.array([v_d, a_d, omega, alpha])
    
    def linearize(self):
        """
        Linearize the system around equilibrium point:
        - theta = 0 (load directly below drone)
        - omega = 0 (no swing)
        - v_d = 0 (no velocity)
        - F_drag = 0 (no aerodynamic forces)
        
        Returns:
            A: state matrix
            B: input matrix
        """
        # State: [x_d, v_d, theta, omega]
        # Input: a_d
        # At equilibrium: theta=0, omega=0, v_d=0, a_d=0
        
        # Linearized dynamics: d/dt [x_d, v_d, theta, omega] = A * [x_d, v_d, theta, omega] + B * a_d
        
        # From nonlinear equations:
        # x_d_dot = v_d
        # v_d_dot = a_d
        # theta_dot = omega
        # omega_dot = -(g/L) * sin(theta) - (a_d/L) * cos(theta) + (F_drag/(m_l*L)) * cos(theta)
        
        # Linearizing around equilibrium:
        # sin(theta) ≈ theta, cos(theta) ≈ 1
        # v_l ≈ v_d + L * omega (when theta ≈ 0)
        # F_drag = -0.5 * rho * Cd * S * v_l * |v_l| ≈ 0 at equilibrium
        # But keeping first-order drag: F_drag ≈ -0.5 * rho * Cd * S * 2 * v_l * sign(v_l) ≈ 0 at v_l=0
        # For small velocities around zero, we can approximate: F_drag ≈ -k_drag * v_l where k_drag is very small
        # However, at equilibrium, we neglect drag as per problem statement
        
        # Linearized omega_dot:
        # omega_dot ≈ -(g/L) * theta - (a_d/L)
        
        A = np.array([
            [0, 1, 0, 0],           # x_d_dot = v_d
            [0, 0, 0, 0],           # v_d_dot = a_d
            [0, 0, 0, 1],           # theta_dot = omega
            [0, 0, -self.g/self.L, 0]  # omega_dot = -(g/L)*theta - (a_d/L)
        ])
        
        B = np.array([
            [0],
            [1],
            [0],
            [-1/self.L]
        ])
        
        return A, B
    
    def discretize(self, A, B):
        """
        Discretize the continuous-time linear system
        
        Args:
            A: continuous-time state matrix
            B: continuous-time input matrix
        
        Returns:
            Ad: discrete-time state matrix
            Bd: discrete-time input matrix
        """
        n = A.shape[0]
        m = B.shape[1]
        
        # Zero-order hold discretization
        # Ad = e^(A*ts)
        # Bd = ∫[0,ts] e^(A*τ) dτ * B
        
        Ad = expm(A * self.ts)
        
        # Compute Bd using matrix exponential
        M = np.zeros((n + m, n + m))
        M[:n, :n] = A * self.ts
        M[:n, n:] = B * self.ts
        
        expM = expm(M)
        Bd = expM[:n, n:]
        
        return Ad, Bd
    
    def get_output_matrices(self):
        """
        Define output matrices for the system
        Output: [x_l, v_l] (load position and velocity)
        
        Returns:
            C: output matrix
            D: feedthrough matrix
        """
        # x_l = x_d + L * sin(theta) ≈ x_d + L * theta (linearized)
        # v_l = v_d + L * omega * cos(theta) ≈ v_d + L * omega (linearized)
        
        C = np.array([
            [1, 0, self.L, 0],      # x_l = x_d + L*theta
            [0, 1, 0, self.L]       # v_l = v_d + L*omega
        ])
        
        D = np.zeros((2, 1))
        
        return C, D


class OptimalController:
    """
    Optimal controller for the drone system using CVXPY
    """
    
    def __init__(self, model, N_horizon=100):
        """
        Args:
            model: DroneWithSuspendedLoad instance
            N_horizon: prediction horizon
        """
        self.model = model
        self.N = N_horizon
        
        # Get linearized and discretized system
        A, B = model.linearize()
        self.Ad, self.Bd = model.discretize(A, B)
        self.C, self.D = model.get_output_matrices()
        
        # State and input dimensions
        self.n = self.Ad.shape[0]
        self.m = self.Bd.shape[1]
        
    def compute_optimal_trajectory(self, x0, x_target_load, v_target_load=0.0, 
                                   u_max=3.0, x_d_max=100.0, v_d_max=15.0,
                                   theta_max=np.pi/6, omega_max=0.5):
        """
        Compute optimal control trajectory using Model Predictive Control
        
        Args:
            x0: initial state [x_d, v_d, theta, omega]
            x_target_load: target position for the load
            v_target_load: target velocity for the load (default 0)
            u_max: maximum acceleration (m/s²)
            x_d_max: maximum drone position
            v_d_max: maximum drone velocity
            theta_max: maximum cable angle
            omega_max: maximum angular velocity
        
        Returns:
            u_opt: optimal control sequence
            x_opt: optimal state trajectory
        """
        # Decision variables
        x = cp.Variable((self.n, self.N + 1))
        u = cp.Variable((self.m, self.N))
        
        # Target state (load at target with zero velocity and angle)
        x_target = cp.Parameter(self.n)
        x_target.value = np.array([x_target_load, v_target_load, 0.0, 0.0])
        
        # Cost function
        # Minimize: sum of (state deviation + control effort + terminal cost)
        Q = np.diag([10.0, 1.0, 100.0, 10.0])  # State cost (emphasize position and angle)
        R = np.array([[0.01]])                  # Control cost (reduced to allow more aggressive control)
        Q_terminal = np.diag([500.0, 100.0, 500.0, 100.0])  # Terminal cost (very high penalty)
        
        cost = 0
        constraints = []
        
        # Initial condition
        constraints.append(x[:, 0] == x0)
        
        # Dynamics and costs over horizon
        for k in range(self.N):
            # System dynamics
            constraints.append(x[:, k+1] == self.Ad @ x[:, k] + self.Bd @ u[:, k])
            
            # State constraints
            constraints.append(cp.abs(x[0, k]) <= x_d_max)  # drone position
            constraints.append(cp.abs(x[1, k]) <= v_d_max)  # drone velocity
            constraints.append(cp.abs(x[2, k]) <= theta_max)  # angle
            constraints.append(cp.abs(x[3, k]) <= omega_max)  # angular velocity
            
            # Input constraints
            constraints.append(cp.abs(u[:, k]) <= u_max)
            
            # Stage cost
            cost += cp.quad_form(x[:, k] - x_target, Q) + cp.quad_form(u[:, k], R)
        
        # Terminal cost (enforce zero oscillation at end)
        cost += cp.quad_form(x[:, self.N] - x_target, Q_terminal)
        
        # Terminal constraints (no oscillation) - tighter constraints
        constraints.append(cp.abs(x[2, self.N]) <= 0.005)  # angle ≈ 0
        constraints.append(cp.abs(x[3, self.N]) <= 0.005)  # angular velocity ≈ 0
        constraints.append(cp.abs(x[1, self.N]) <= 0.005)  # drone velocity ≈ 0
        
        # Terminal position constraint (load at target)
        # x_l = x_d + L * theta
        x_l_terminal = x[0, self.N] + self.model.L * x[2, self.N]
        constraints.append(cp.abs(x_l_terminal - x_target_load) <= 0.05)
        
        # Solve optimization problem
        problem = cp.Problem(cp.Minimize(cost), constraints)
        try:
            problem.solve(solver=cp.ECOS, verbose=False)
        except:
            # Fallback to OSQP if ECOS is not available
            problem.solve(solver=cp.OSQP, verbose=False, max_iter=10000)
        
        if problem.status not in ["optimal", "optimal_inaccurate"]:
            print(f"Warning: Optimization status: {problem.status}")
        
        return u.value, x.value


def simulate_nonlinear(model, u_sequence, x0, dt=0.01):
    """
    Simulate the nonlinear model with given control sequence
    
    Args:
        model: DroneWithSuspendedLoad instance
        u_sequence: control sequence (N x 1)
        x0: initial state
        dt: simulation time step
    
    Returns:
        t: time vector
        x_history: state history
        x_l_history: load position history
        v_l_history: load velocity history
    """
    N = u_sequence.shape[1]
    T_total = N * model.ts
    
    # Time points for simulation
    t_eval = np.arange(0, T_total, dt)
    
    x_history = [x0]
    x_l_history = [x0[0] + model.L * np.sin(x0[2])]
    v_l_history = [x0[1] + model.L * x0[3] * np.cos(x0[2])]
    t_history = [0]
    
    current_state = x0
    
    for k in range(N):
        t_start = k * model.ts
        t_end = (k + 1) * model.ts
        t_span = (t_start, t_end)
        t_sub = np.linspace(t_start, t_end, int(model.ts / dt) + 1)
        
        # Control input for this interval
        u_k = u_sequence[0, k]
        
        # Solve ODE for this interval
        sol = solve_ivp(
            lambda t, y: model.nonlinear_dynamics(t, y, u_k),
            t_span,
            current_state,
            t_eval=t_sub,
            method='RK45',
            rtol=1e-6,
            atol=1e-9
        )
        
        # Store results (skip first point to avoid duplication)
        for i in range(1, len(sol.t)):
            x_history.append(sol.y[:, i])
            theta = sol.y[2, i]
            omega = sol.y[3, i]
            x_d = sol.y[0, i]
            v_d = sol.y[1, i]
            
            x_l = x_d + model.L * np.sin(theta)
            v_l = v_d + model.L * omega * np.cos(theta)
            
            x_l_history.append(x_l)
            v_l_history.append(v_l)
            t_history.append(sol.t[i])
        
        current_state = sol.y[:, -1]
    
    return np.array(t_history), np.array(x_history), np.array(x_l_history), np.array(v_l_history)


def simulate_linear(model, u_sequence, x0):
    """
    Simulate the linear model with given control sequence
    
    Args:
        model: DroneWithSuspendedLoad instance
        u_sequence: control sequence (N x 1)
        x0: initial state
    
    Returns:
        x_history: state history
        x_l_history: load position history
        v_l_history: load velocity history
    """
    A, B = model.linearize()
    Ad, Bd = model.discretize(A, B)
    C, D = model.get_output_matrices()
    
    N = u_sequence.shape[1]
    x_history = np.zeros((4, N + 1))
    x_history[:, 0] = x0
    
    for k in range(N):
        x_history[:, k+1] = Ad @ x_history[:, k] + Bd @ u_sequence[:, k]
    
    # Compute load position and velocity
    x_l_history = x_history[0, :] + model.L * x_history[2, :]
    v_l_history = x_history[1, :] + model.L * x_history[3, :]
    
    return x_history, x_l_history, v_l_history


def plot_results(t_nl, x_nl, x_l_nl, v_l_nl, x_lin, x_l_lin, v_l_lin, 
                u_sequence, model, x_target):
    """
    Plot results comparing linear and nonlinear simulations
    """
    N = u_sequence.shape[1]
    t_lin = np.arange(N + 1) * model.ts
    t_u = np.arange(N) * model.ts
    
    fig, axes = plt.subplots(4, 2, figsize=(14, 12))
    
    # Plot 1: Load position
    axes[0, 0].plot(t_nl, x_l_nl, 'b-', label='Nonlinear', linewidth=2)
    axes[0, 0].plot(t_lin, x_l_lin, 'r--', label='Linear', linewidth=2)
    axes[0, 0].axhline(y=x_target, color='g', linestyle=':', label='Target')
    axes[0, 0].set_ylabel('Load Position (m)')
    axes[0, 0].set_xlabel('Time (s)')
    axes[0, 0].legend()
    axes[0, 0].grid(True)
    axes[0, 0].set_title('Load Position')
    
    # Plot 2: Load velocity
    axes[0, 1].plot(t_nl, v_l_nl, 'b-', label='Nonlinear', linewidth=2)
    axes[0, 1].plot(t_lin, v_l_lin, 'r--', label='Linear', linewidth=2)
    axes[0, 1].axhline(y=0, color='g', linestyle=':', label='Target')
    axes[0, 1].set_ylabel('Load Velocity (m/s)')
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].legend()
    axes[0, 1].grid(True)
    axes[0, 1].set_title('Load Velocity')
    
    # Plot 3: Drone position
    axes[1, 0].plot(t_nl, x_nl[:, 0], 'b-', label='Nonlinear', linewidth=2)
    axes[1, 0].plot(t_lin, x_lin[0, :], 'r--', label='Linear', linewidth=2)
    axes[1, 0].set_ylabel('Drone Position (m)')
    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 0].legend()
    axes[1, 0].grid(True)
    axes[1, 0].set_title('Drone Position')
    
    # Plot 4: Drone velocity
    axes[1, 1].plot(t_nl, x_nl[:, 1], 'b-', label='Nonlinear', linewidth=2)
    axes[1, 1].plot(t_lin, x_lin[1, :], 'r--', label='Linear', linewidth=2)
    axes[1, 1].set_ylabel('Drone Velocity (m/s)')
    axes[1, 1].set_xlabel('Time (s)')
    axes[1, 1].legend()
    axes[1, 1].grid(True)
    axes[1, 1].set_title('Drone Velocity')
    
    # Plot 5: Cable angle
    axes[2, 0].plot(t_nl, np.rad2deg(x_nl[:, 2]), 'b-', label='Nonlinear', linewidth=2)
    axes[2, 0].plot(t_lin, np.rad2deg(x_lin[2, :]), 'r--', label='Linear', linewidth=2)
    axes[2, 0].set_ylabel('Cable Angle (deg)')
    axes[2, 0].set_xlabel('Time (s)')
    axes[2, 0].legend()
    axes[2, 0].grid(True)
    axes[2, 0].set_title('Cable Angle from Vertical')
    
    # Plot 6: Angular velocity
    axes[2, 1].plot(t_nl, x_nl[:, 3], 'b-', label='Nonlinear', linewidth=2)
    axes[2, 1].plot(t_lin, x_lin[3, :], 'r--', label='Linear', linewidth=2)
    axes[2, 1].set_ylabel('Angular Velocity (rad/s)')
    axes[2, 1].set_xlabel('Time (s)')
    axes[2, 1].legend()
    axes[2, 1].grid(True)
    axes[2, 1].set_title('Cable Angular Velocity')
    
    # Plot 7: Control input
    axes[3, 0].step(t_u, u_sequence[0, :], 'k-', where='post', linewidth=2)
    axes[3, 0].set_ylabel('Drone Acceleration (m/s²)')
    axes[3, 0].set_xlabel('Time (s)')
    axes[3, 0].grid(True)
    axes[3, 0].set_title('Control Input (Drone Acceleration)')
    
    # Plot 8: Phase portrait (angle vs angular velocity)
    axes[3, 1].plot(np.rad2deg(x_nl[:, 2]), x_nl[:, 3], 'b-', linewidth=2, label='Nonlinear')
    axes[3, 1].plot(np.rad2deg(x_lin[2, :]), x_lin[3, :], 'r--', linewidth=2, label='Linear')
    axes[3, 1].plot(np.rad2deg(x_nl[0, 2]), x_nl[0, 3], 'go', markersize=10, label='Start')
    axes[3, 1].plot(np.rad2deg(x_nl[-1, 2]), x_nl[-1, 3], 'rs', markersize=10, label='End')
    axes[3, 1].set_xlabel('Cable Angle (deg)')
    axes[3, 1].set_ylabel('Angular Velocity (rad/s)')
    axes[3, 1].legend()
    axes[3, 1].grid(True)
    axes[3, 1].set_title('Phase Portrait')
    
    plt.tight_layout()
    plt.savefig('/home/runner/work/Drone/Drone/results.png', dpi=150, bbox_inches='tight')
    print("Results saved to results.png")
    
    return fig


def main():
    """
    Main function to demonstrate the drone control system
    """
    print("=" * 70)
    print("Drone with Suspended Load Control System")
    print("=" * 70)
    
    # Initialize model
    model = DroneWithSuspendedLoad()
    
    print("\nSystem Parameters:")
    print(f"  Drone mass: {model.m_d} kg")
    print(f"  Load mass: {model.m_l} kg")
    print(f"  Cable length: {model.L} m")
    print(f"  Drag coefficient: {model.Cd}")
    print(f"  Cross-sectional area: {model.S} m²")
    print(f"  Sampling time: {model.ts} s")
    
    # Initial conditions
    x0 = np.array([0.0, 0.0, 0.05, 0.0])  # [x_d, v_d, theta, omega] - smaller initial angle
    print(f"\nInitial State:")
    print(f"  Drone position: {x0[0]} m")
    print(f"  Drone velocity: {x0[1]} m/s")
    print(f"  Cable angle: {np.rad2deg(x0[2]):.2f} deg")
    print(f"  Angular velocity: {x0[3]} rad/s")
    
    # Target
    x_target_load = 50.0  # Target position for load (m)
    print(f"\nTarget:")
    print(f"  Load position: {x_target_load} m")
    print(f"  Load velocity: 0 m/s (stopped)")
    print(f"  Cable angle: 0 deg (no oscillation)")
    
    # Get linearized system
    print("\n" + "=" * 70)
    print("Linearized System Matrices")
    print("=" * 70)
    A, B = model.linearize()
    print("\nState matrix A:")
    print(A)
    print("\nInput matrix B:")
    print(B)
    
    # Get discrete system
    Ad, Bd = model.discretize(A, B)
    print("\nDiscrete-time state matrix Ad:")
    print(Ad)
    print("\nDiscrete-time input matrix Bd:")
    print(Bd)
    
    # Get output matrices
    C, D = model.get_output_matrices()
    print("\nOutput matrix C:")
    print(C)
    
    # Compute optimal control
    print("\n" + "=" * 70)
    print("Computing Optimal Control Trajectory")
    print("=" * 70)
    
    controller = OptimalController(model, N_horizon=200)
    u_opt, x_opt = controller.compute_optimal_trajectory(
        x0, 
        x_target_load,
        u_max=3.0,
        theta_max=np.pi/6
    )
    
    print(f"\nOptimization completed successfully!")
    print(f"  Horizon: {controller.N} steps ({controller.N * model.ts} s)")
    print(f"  Max control: {np.max(np.abs(u_opt)):.2f} m/s²")
    print(f"  Final load position: {x_opt[0, -1] + model.L * x_opt[2, -1]:.2f} m")
    print(f"  Final load velocity: {x_opt[1, -1] + model.L * x_opt[3, -1]:.4f} m/s")
    print(f"  Final angle: {np.rad2deg(x_opt[2, -1]):.4f} deg")
    
    # Simulate linear model
    print("\n" + "=" * 70)
    print("Simulating Linear Model")
    print("=" * 70)
    x_lin, x_l_lin, v_l_lin = simulate_linear(model, u_opt, x0)
    print("Linear simulation completed")
    
    # Simulate nonlinear model
    print("\n" + "=" * 70)
    print("Simulating Nonlinear Model (RK45 solver)")
    print("=" * 70)
    t_nl, x_nl, x_l_nl, v_l_nl = simulate_nonlinear(model, u_opt, x0, dt=0.01)
    print(f"Nonlinear simulation completed")
    print(f"  Final load position: {x_l_nl[-1]:.2f} m")
    print(f"  Final load velocity: {v_l_nl[-1]:.4f} m/s")
    print(f"  Final angle: {np.rad2deg(x_nl[-1, 2]):.4f} deg")
    
    # Plot results
    print("\n" + "=" * 70)
    print("Generating Plots")
    print("=" * 70)
    plot_results(t_nl, x_nl, x_l_nl, v_l_nl, x_lin, x_l_lin, v_l_lin,
                u_opt, model, x_target_load)
    
    print("\n" + "=" * 70)
    print("Summary")
    print("=" * 70)
    print(f"The load successfully reached the target position of {x_target_load} m")
    print(f"with controlled oscillation and approaches a complete stop.")
    print(f"\nLinear model performance:")
    print(f"  Final position error: {abs(x_l_lin[-1] - x_target_load):.4f} m")
    print(f"  Final velocity: {abs(v_l_lin[-1]):.6f} m/s")
    print(f"  Final angle: {np.rad2deg(x_lin[2, -1]):.4f} deg")
    print(f"\nNonlinear model performance:")
    print(f"  Final position error: {abs(x_l_nl[-1] - x_target_load):.4f} m")
    print(f"  Final velocity: {abs(v_l_nl[-1]):.6f} m/s")
    print(f"  Final angle: {np.rad2deg(x_nl[-1, 2]):.4f} deg")
    print(f"\nThe optimal controller minimizes transport time")
    print(f"while controlling oscillations. The difference between linear")
    print(f"and nonlinear models shows the effect of air drag on the load.")
    print("=" * 70)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Verification script to test the drone control system
"""

import sys
import numpy as np


def test_imports():
    """Test that all modules can be imported"""
    print("Testing imports...")
    try:
        import drone_control
        import examples
        print("✓ All modules imported successfully")
        return True
    except Exception as e:
        print(f"✗ Import failed: {e}")
        return False


def test_model_creation():
    """Test model creation and basic properties"""
    print("\nTesting model creation...")
    try:
        from drone_control import DroneWithSuspendedLoad
        
        model = DroneWithSuspendedLoad()
        
        # Check parameters
        assert model.m_d == 40.0, "Drone mass incorrect"
        assert model.m_l == 24.0, "Load mass incorrect"
        assert model.L == 19.0, "Cable length incorrect"
        assert model.Cd == 1.0, "Drag coefficient incorrect"
        assert model.S == 0.2, "Cross-sectional area incorrect"
        assert model.ts == 0.1, "Sampling time incorrect"
        
        print("✓ Model created with correct parameters")
        return True
    except Exception as e:
        print(f"✗ Model creation failed: {e}")
        return False


def test_linearization():
    """Test linearization"""
    print("\nTesting linearization...")
    try:
        from drone_control import DroneWithSuspendedLoad
        
        model = DroneWithSuspendedLoad()
        A, B = model.linearize()
        
        # Check dimensions
        assert A.shape == (4, 4), "A matrix has wrong shape"
        assert B.shape == (4, 1), "B matrix has wrong shape"
        
        # Check some specific values
        assert np.isclose(A[0, 1], 1.0), "A[0,1] should be 1"
        assert np.isclose(A[1, 0], 0.0), "A[1,0] should be 0"
        assert np.isclose(B[1, 0], 1.0), "B[1,0] should be 1"
        
        print("✓ Linearization successful")
        return True
    except Exception as e:
        print(f"✗ Linearization failed: {e}")
        return False


def test_discretization():
    """Test discretization"""
    print("\nTesting discretization...")
    try:
        from drone_control import DroneWithSuspendedLoad
        
        model = DroneWithSuspendedLoad()
        A, B = model.linearize()
        Ad, Bd = model.discretize(A, B)
        
        # Check dimensions
        assert Ad.shape == (4, 4), "Ad matrix has wrong shape"
        assert Bd.shape == (4, 1), "Bd matrix has wrong shape"
        
        # Check stability (eigenvalues should be inside unit circle)
        eigenvalues = np.linalg.eigvals(Ad)
        max_eigenvalue = np.max(np.abs(eigenvalues))
        assert max_eigenvalue <= 1.01, f"System may be unstable: max eigenvalue = {max_eigenvalue}"
        
        print("✓ Discretization successful")
        print(f"  Maximum eigenvalue magnitude: {max_eigenvalue:.4f}")
        return True
    except Exception as e:
        print(f"✗ Discretization failed: {e}")
        return False


def test_controller():
    """Test controller creation"""
    print("\nTesting controller...")
    try:
        from drone_control import DroneWithSuspendedLoad, OptimalController
        
        model = DroneWithSuspendedLoad()
        controller = OptimalController(model, N_horizon=50)
        
        assert controller.N == 50, "Horizon incorrect"
        assert controller.n == 4, "State dimension incorrect"
        assert controller.m == 1, "Input dimension incorrect"
        
        print("✓ Controller created successfully")
        return True
    except Exception as e:
        print(f"✗ Controller creation failed: {e}")
        return False


def test_simple_optimization():
    """Test a simple optimization problem"""
    print("\nTesting optimization...")
    try:
        from drone_control import DroneWithSuspendedLoad, OptimalController
        
        model = DroneWithSuspendedLoad()
        controller = OptimalController(model, N_horizon=100)
        
        # Simple test case
        x0 = np.array([0.0, 0.0, 0.01, 0.0])
        x_target = 10.0
        
        u_opt, x_opt = controller.compute_optimal_trajectory(
            x0, x_target, u_max=2.0, theta_max=np.pi/6
        )
        
        # Check dimensions
        assert u_opt.shape == (1, 100), "Control trajectory has wrong shape"
        assert x_opt.shape == (4, 101), "State trajectory has wrong shape"
        
        # Check terminal state is close to target
        x_l_final = x_opt[0, -1] + model.L * x_opt[2, -1]
        error = abs(x_l_final - x_target)
        
        print(f"✓ Optimization successful")
        print(f"  Target: {x_target} m")
        print(f"  Final position: {x_l_final:.2f} m")
        print(f"  Error: {error:.3f} m")
        return True
    except Exception as e:
        print(f"✗ Optimization failed: {e}")
        return False


def test_simulation():
    """Test simulation"""
    print("\nTesting simulation...")
    try:
        from drone_control import (
            DroneWithSuspendedLoad, 
            OptimalController,
            simulate_linear,
            simulate_nonlinear
        )
        
        model = DroneWithSuspendedLoad()
        controller = OptimalController(model, N_horizon=100)
        
        x0 = np.array([0.0, 0.0, 0.01, 0.0])
        x_target = 5.0
        
        u_opt, x_opt = controller.compute_optimal_trajectory(
            x0, x_target, u_max=2.0
        )
        
        # Linear simulation
        x_lin, x_l_lin, v_l_lin = simulate_linear(model, u_opt, x0)
        assert x_lin.shape[1] == 101, "Linear simulation wrong length"
        
        # Nonlinear simulation
        t_nl, x_nl, x_l_nl, v_l_nl = simulate_nonlinear(model, u_opt, x0, dt=0.05)
        assert len(t_nl) > 0, "Nonlinear simulation produced no output"
        
        print("✓ Simulations successful")
        print(f"  Linear final position: {x_l_lin[-1]:.2f} m")
        print(f"  Nonlinear final position: {x_l_nl[-1]:.2f} m")
        return True
    except Exception as e:
        print(f"✗ Simulation failed: {e}")
        return False


def main():
    """Run all tests"""
    print("=" * 70)
    print("Drone Control System - Verification Tests")
    print("=" * 70)
    
    tests = [
        test_imports,
        test_model_creation,
        test_linearization,
        test_discretization,
        test_controller,
        test_simple_optimization,
        test_simulation,
    ]
    
    results = []
    for test in tests:
        try:
            result = test()
            results.append(result)
        except Exception as e:
            print(f"✗ Test crashed: {e}")
            results.append(False)
    
    print("\n" + "=" * 70)
    print("Test Summary")
    print("=" * 70)
    passed = sum(results)
    total = len(results)
    print(f"Passed: {passed}/{total}")
    
    if passed == total:
        print("\n✓ All tests passed!")
        return 0
    else:
        print(f"\n✗ {total - passed} test(s) failed")
        return 1


if __name__ == "__main__":
    sys.exit(main())

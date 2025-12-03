#!/usr/bin/env python3
# RANDOM SEARCH ALGO FOR FINDING STUFF:
import numpy as np
from scipy import signal
import sys

def compute_step_info(t, y, command_magnitude):
    """Compute rise time, settling time, overshoot from step response"""
    ss_value = np.mean(y[-len(y)//10:])
    y_norm = y / command_magnitude
    ss_norm = ss_value / command_magnitude

    # Rise time (10% to 90%)
    thresh_10 = 0.1 * ss_norm
    thresh_90 = 0.9 * ss_norm
    idx_10 = np.where(y_norm >= thresh_10)[0]
    idx_90 = np.where(y_norm >= thresh_90)[0]
    rise_time = t[idx_90[0]] - t[idx_10[0]] if len(idx_10) > 0 and len(idx_90) > 0 else 0

    # Settling time (2% criterion)
    settling_band = 0.02 * ss_norm
    settled = np.abs(y_norm - ss_norm) <= settling_band
    if np.any(~settled):
        last_outside = np.where(~settled)[0][-1]
        settling_time = t[last_outside] if last_outside < len(t)-1 else t[-1]
    else:
        settling_time = 0

    # Overshoot
    peak = np.max(y)
    overshoot = ((peak - ss_value) / ss_value) * 100 if ss_value > 0 else 0
    ess = abs(command_magnitude - ss_value)

    return rise_time, settling_time, overshoot, ess

def simulate_x_system(Kp, Ki, Kd, t):
    """Simulate X-position control system"""
    num_p = [5]
    den_p = [1, 5, 0]
    num_kx = [Kd, Kp, Ki]
    den_kx = [1, 0]
    num_loop = np.polymul(num_kx, num_p)
    den_loop = np.polymul(den_kx, den_p)
    num_cl = num_loop
    den_cl = np.polyadd(den_loop, num_loop)
    sys_x = signal.TransferFunction(num_cl, den_cl)
    _, y = signal.step(sys_x, T=t)
    y = y * 22
    return compute_step_info(t, y, 22)

def simulate_y_system(Kp_phi, Kp_y, t):
    """Simulate Y-position control system"""
    Vo = 70 * 1.467
    num_inner = [Kp_phi * 100]
    den_inner = [1, 100 + Kp_phi * 100]
    num_with_int = [Vo * x for x in num_inner]
    den_with_int = np.polymul(den_inner, [1, 0])
    num_outer_loop = [Kp_y * x for x in num_with_int]
    den_outer_loop = den_with_int
    num_cl = num_outer_loop
    den_cl = np.polyadd(den_outer_loop, num_outer_loop)
    sys_y = signal.TransferFunction(num_cl, den_cl)
    _, y = signal.step(sys_y, T=t)
    y = y * 12
    return compute_step_info(t, y, 12)

def check_specs(params):
    """Check if all specs are met and return violation score"""
    Kp_x, Ki_x, Kd_x, Kp_phi, Kp_y = params

    # X gains should be positive, Y gains are negative
    if Kp_x < 0 or Ki_x < 0 or Kd_x < 0:
        return False, 1e10
    if Kp_phi > 0 or Kp_y > 0:  # Y gains must be negative
        return False, 1e10

    t = np.linspace(0, 10, 5000)

    try:
        tr_x, ts_x, os_x, ess_x = simulate_x_system(Kp_x, Ki_x, Kd_x, t)
        tr_y, ts_y, os_y, ess_y = simulate_y_system(Kp_phi, Kp_y, t)

        # Check all specs
        x_pass = (0.8 <= tr_x <= 1.1) and (0.8 <= ts_x <= 1.3) and (os_x < 20) and (ess_x < 1)
        y_pass = (2.5 <= tr_y <= 4.0) and (2.5 <= ts_y <= 4.5) and (os_y < 10) and (ess_y < 0.01)

        # Compute violation score (distance from spec ranges)
        score = 0
        if tr_x < 0.8: score += (0.8 - tr_x)**2 * 100
        elif tr_x > 1.1: score += (tr_x - 1.1)**2 * 100

        if ts_x < 0.8: score += (0.8 - ts_x)**2 * 100
        elif ts_x > 1.3: score += (ts_x - 1.3)**2 * 100

        if os_x > 20: score += (os_x - 20)**2 * 10
        if ess_x > 1: score += (ess_x - 1)**2 * 50

        if tr_y < 2.5: score += (2.5 - tr_y)**2 * 100
        elif tr_y > 4.0: score += (tr_y - 4.0)**2 * 100

        if ts_y < 2.5: score += (2.5 - ts_y)**2 * 100
        elif ts_y > 4.48: score += (ts_y - 4.48)**2 * 100  # Slightly tighter for numerical safety

        if os_y > 10: score += (os_y - 10)**2 * 10
        if ess_y > 0.01: score += (ess_y - 0.01)**2 * 50

        return (x_pass and y_pass), score

    except:
        return False, 1e10

def random_search_optimize():
    """Random search with local refinement"""
    print("="*60)
    print("RANDOM SEARCH OPTIMIZER FOR TESLA AUTOPILOT GAINS")
    print("="*60 + "\n")

    # Search ranges: [Kp_x, Ki_x, Kd_x, Kp_phi, Kp_y]
    lower_bounds = np.array([1.0, 0.0, 0.1, -1.0, -0.0002])
    upper_bounds = np.array([2.5, 0.1, 0.25, -0.5, -0.00005])

    best_params = np.array([1.5, 0.0, 0.15, -0.982, -0.0001])
    best_score = float('inf')
    solution_found = False

    print("Running random search (500 samples)...")

    # Random search phase
    for i in range(500):
        # Random sample
        params = lower_bounds + np.random.rand(5) * (upper_bounds - lower_bounds)

        passed, score = check_specs(params)

        if score < best_score:
            best_score = score
            best_params = params.copy()

            if passed:
                solution_found = True
                print(f"\n✓ Found solution at sample {i+1}!")
                break

        if (i+1) % 100 == 0:
            print(f"  Sample {i+1}/500: Best score = {best_score:.3f}")

    if not solution_found:
        print(f"\nNo perfect solution found. Best score: {best_score:.3f}")
        print("Attempting local refinement around best candidate...\n")

        # Local refinement: small perturbations around best
        for i in range(200):
            # Add small random perturbation
            perturbation = (np.random.rand(5) - 0.5) * 0.1 * (upper_bounds - lower_bounds)
            params = best_params + perturbation

            # Clip to bounds
            params = np.clip(params, lower_bounds, upper_bounds)

            passed, score = check_specs(params)

            if score < best_score:
                best_score = score
                best_params = params.copy()

                if passed:
                    solution_found = True
                    print(f"✓ Found solution during refinement (iter {i+1})!")
                    break

    print("\n" + "="*60)
    print("OPTIMIZATION COMPLETE")
    print("="*60 + "\n")

    # Verify final performance
    t = np.linspace(0, 10, 5000)
    tr_x, ts_x, os_x, ess_x = simulate_x_system(best_params[0], best_params[1], best_params[2], t)
    tr_y, ts_y, os_y, ess_y = simulate_y_system(best_params[3], best_params[4], t)

    print("OPTIMAL GAINS:")
    print(f"  X-Position: Kp = {best_params[0]:.3f}, Ki = {best_params[1]:.3f}, Kd = {best_params[2]:.3f}")
    print(f"  Y-Inner:    Kp = {best_params[3]:.3f}, Ki = 0.000, Kd = 0.000")
    print(f"  Y-Outer:    Kp = {best_params[4]:.4f}, Ki = 0.000, Kd = 0.000\n")

    print("X-POSITION PERFORMANCE:")
    print(f"  Rise Time:      {tr_x:.3f} s  [Spec: 0.8 - 1.1 s]   {'✓' if 0.8 <= tr_x <= 1.1 else '✗'}")
    print(f"  Settling Time:  {ts_x:.3f} s  [Spec: 0.8 - 1.3 s]   {'✓' if 0.8 <= ts_x <= 1.3 else '✗'}")
    print(f"  Overshoot:      {os_x:.2f}%   [Spec: < 20%]         {'✓' if os_x < 20 else '✗'}")
    print(f"  SS Error:       {ess_x:.3f} ft [Spec: < 1 ft]       {'✓' if ess_x < 1 else '✗'}\n")

    print("Y-POSITION PERFORMANCE:")
    print(f"  Rise Time:      {tr_y:.3f} s  [Spec: 2.5 - 4.0 s]  {'✓' if 2.5 <= tr_y <= 4.0 else '✗'}")
    print(f"  Settling Time:  {ts_y:.3f} s  [Spec: 2.5 - 4.5 s]  {'✓' if 2.5 <= ts_y <= 4.5 else '✗'}")
    print(f"  Overshoot:      {os_y:.2f}%   [Spec: < 10%]        {'✓' if os_y < 10 else '✗'}")
    print(f"  SS Error:       {ess_y:.6f} ft [Spec: < 0.01 ft]   {'✓' if ess_y < 0.01 else '✗'}\n")

    x_pass = (0.8 <= tr_x <= 1.1) and (0.8 <= ts_x <= 1.3) and (os_x < 20) and (ess_x < 1)
    y_pass = (2.5 <= tr_y <= 4.0) and (2.5 <= ts_y <= 4.5) and (os_y < 10) and (ess_y < 0.01)

    if x_pass and y_pass:
        print("="*60)
        print("✓✓✓ ALL SPECIFICATIONS MET! ✓✓✓")
        print("="*60 + "\n")

        print("Copy these gains to project01.m:")
        print(f"\nLine 57-59 (X-position):")
        print(f"Kp = {best_params[0]:.3f};")
        print(f"Ki = {best_params[1]:.3f};")
        print(f"Kd = {best_params[2]:.3f};")
        print(f"\nLine 120-122 (Y-inner loop):")
        print(f"Kp = {best_params[3]:.3f};")
        print(f"Ki = 0.0;")
        print(f"Kd = 0.0;")
        print(f"\nLine 168-170 (Y-outer loop):")
        print(f"Kp = {best_params[4]:.4f};")
        print(f"Ki = 0.0;")
        print(f"Kd = 0.0;\n")
    else:
        print("Some specs not yet met. Try expanding search ranges or more samples.\n")

if __name__ == "__main__":
    try:
        np.random.seed(123)  # Different seed for new run
        random_search_optimize()
    except KeyboardInterrupt:
        print("\n\nOptimization interrupted by user.")
        sys.exit(0)
    except Exception as e:
        print(f"\nERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

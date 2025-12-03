#!/usr/bin/env python3
import numpy as np
from scipy import signal
import sys

def get_status(passed):
    return "PASS ✓" if passed else "FAIL ✗"

def get_overall_status(passed):
    return "ALL SPECS MET ✓" if passed else "SPECS VIOLATED ✗"

def compute_step_info(t, y, command_magnitude):
    """Compute rise time, settling time, overshoot from step response"""
    # Find steady-state value (last 10% of response)
    ss_value = np.mean(y[-len(y)//10:])

    # Normalize to unit step
    y_norm = y / command_magnitude
    ss_norm = ss_value / command_magnitude

    # Rise time (10% to 90% of steady-state)
    thresh_10 = 0.1 * ss_norm
    thresh_90 = 0.9 * ss_norm

    idx_10 = np.where(y_norm >= thresh_10)[0]
    idx_90 = np.where(y_norm >= thresh_90)[0]

    if len(idx_10) > 0 and len(idx_90) > 0:
        rise_time = t[idx_90[0]] - t[idx_10[0]]
    else:
        rise_time = 0

    # Settling time (2% criterion)
    settling_band = 0.02 * ss_norm
    settled = np.abs(y_norm - ss_norm) <= settling_band

    # Find last time it left the band
    if np.any(~settled):
        last_outside = np.where(~settled)[0][-1]
        settling_time = t[last_outside] if last_outside < len(t)-1 else t[-1]
    else:
        settling_time = 0

    # Overshoot
    peak = np.max(y)
    overshoot = ((peak - ss_value) / ss_value) * 100 if ss_value > 0 else 0

    return {
        'RiseTime': rise_time,
        'SettlingTime': settling_time,
        'Overshoot': overshoot,
        'Peak': peak,
        'SteadyStateValue': ss_value
    }

def verify_specs():
    print("\n" + "="*40)
    print("TESLA AUTOPILOT PERFORMANCE VERIFICATION")
    print("="*40 + "\n")

    # X-Position gains (from project01.m lines 57-59)
    Kp_x, Ki_x, Kd_x = 2.153, 0.096, 0.133

    # Y-Position gains (from project01.m)
    Kp_phi = -0.981  # Inner loop (line 120)
    Kp_y = -0.0001   # Outer loop (line 168)

    Vo = 70 * 1.467  # 70 mph to ft/s

    # Time vector
    t = np.linspace(0, 10, 10000)

    # ========== X-POSITION CONTROL ==========
    print("--- X-POSITION CONTROL ---")
    print(f"Gains: Kp = {Kp_x:.2f}, Ki = {Ki_x:.3f}, Kd = {Kd_x:.3f}\n")

    # Plant: P(s) = 5/(s^2 + 5s)
    num_p = [5]
    den_p = [1, 5, 0]

    # PID Controller: Kx(s) = Kd*s + Kp + Ki/s = (Kd*s^2 + Kp*s + Ki)/s
    num_kx = [Kd_x, Kp_x, Ki_x]
    den_kx = [1, 0]

    # Loop gain: Kx * P
    num_loop = np.polymul(num_kx, num_p)
    den_loop = np.polymul(den_kx, den_p)

    # Closed-loop: T(s) = L(s)/(1 + L(s))
    num_cl_x = num_loop
    den_cl_x = np.polyadd(den_loop, num_loop)

    # Step response for 22 ft command
    sys_x = signal.TransferFunction(num_cl_x, den_cl_x)
    _, y_x = signal.step(sys_x, T=t)
    y_x = y_x * 22  # Scale to 22 ft command

    # Compute metrics
    info_x = compute_step_info(t, y_x, 22)
    ess_x = abs(22 - info_x['SteadyStateValue'])

    print("Performance Metrics:")
    print(f"  Rise Time (tr):        {info_x['RiseTime']:.3f} s   [Spec: 0.8 < tr < 1.1 s]")
    print(f"  Settling Time (ts):    {info_x['SettlingTime']:.3f} s   [Spec: 0.8 < ts < 1.3 s]")
    print(f"  Overshoot:             {info_x['Overshoot']:.2f}%   [Spec: < 20%]")
    print(f"  Peak:                  {info_x['Peak']:.3f} ft  [Command: 22 ft]")
    print(f"  Steady-State Value:    {info_x['SteadyStateValue']:.3f} ft")
    print(f"  Steady-State Error:    {ess_x:.3f} ft  [Spec: < 1 ft]")

    # Check specs
    pass_x_tr = 0.8 <= info_x['RiseTime'] <= 1.1
    pass_x_ts = 0.8 <= info_x['SettlingTime'] <= 1.3
    pass_x_os = info_x['Overshoot'] < 20
    pass_x_ess = ess_x < 1

    print("\nSpecification Check:")
    print(f"  Rise Time:      {get_status(pass_x_tr)}")
    print(f"  Settling Time:  {get_status(pass_x_ts)}")
    print(f"  Overshoot:      {get_status(pass_x_os)}")
    print(f"  SS Error:       {get_status(pass_x_ess)}")

    x_all_pass = pass_x_tr and pass_x_ts and pass_x_os and pass_x_ess
    print(f"\n  X-POSITION: {get_overall_status(x_all_pass)}")

    # ========== Y-POSITION CONTROL ==========
    print("\n\n--- Y-POSITION CONTROL ---")
    print(f"Inner Loop: Kp = {Kp_phi:.2f}, Ki = 0.000, Kd = 0.000")
    print(f"Outer Loop: Kp = {Kp_y:.3f}, Ki = 0.000, Kd = 0.000\n")

    # Inner loop: phi/phi_c = Kphi * 100/(s+100) / (1 + Kphi * 100/(s+100))
    #                       = Kphi*100 / (s + 100 + Kphi*100)
    num_inner = [Kp_phi * 100]
    den_inner = [1, 100 + Kp_phi * 100]

    # Add integrator: Vo/s
    num_with_int = num_inner  # numerator stays same
    den_with_int = np.polymul(den_inner, [1, 0])  # multiply denominator by s

    # Multiply by Vo
    num_with_int = [Vo * x for x in num_with_int]

    # Outer loop with Kp_y
    num_outer_loop = [Kp_y * x for x in num_with_int]
    den_outer_loop = den_with_int

    # Closed-loop: T(s) = L(s)/(1 + L(s))
    num_cl_y = num_outer_loop
    den_cl_y = np.polyadd(den_outer_loop, num_outer_loop)

    # Step response for 12 ft command
    sys_y = signal.TransferFunction(num_cl_y, den_cl_y)
    _, y_y = signal.step(sys_y, T=t)
    y_y = y_y * 12  # Scale to 12 ft command

    # Compute metrics
    info_y = compute_step_info(t, y_y, 12)
    ess_y = abs(12 - info_y['SteadyStateValue'])

    print("Performance Metrics:")
    print(f"  Rise Time (tr):        {info_y['RiseTime']:.3f} s   [Spec: 2.5 < tr < 4.0 s]")
    print(f"  Settling Time (ts):    {info_y['SettlingTime']:.3f} s   [Spec: 2.5 < ts < 4.5 s]")
    print(f"  Overshoot:             {info_y['Overshoot']:.2f}%   [Spec: < 10%]")
    print(f"  Peak:                  {info_y['Peak']:.3f} ft  [Command: 12 ft]")
    print(f"  Steady-State Value:    {info_y['SteadyStateValue']:.3f} ft")
    print(f"  Steady-State Error:    {ess_y:.6f} ft  [Spec: 0 ft]")

    # Check specs
    pass_y_tr = 2.5 <= info_y['RiseTime'] <= 4.0
    pass_y_ts = 2.5 <= info_y['SettlingTime'] <= 4.5
    pass_y_os = info_y['Overshoot'] < 10
    pass_y_ess = ess_y < 0.01

    print("\nSpecification Check:")
    print(f"  Rise Time:      {get_status(pass_y_tr)}")
    print(f"  Settling Time:  {get_status(pass_y_ts)}")
    print(f"  Overshoot:      {get_status(pass_y_os)}")
    print(f"  SS Error:       {get_status(pass_y_ess)}")

    y_all_pass = pass_y_tr and pass_y_ts and pass_y_os and pass_y_ess
    print(f"\n  Y-POSITION: {get_overall_status(y_all_pass)}")

    # ========== OVERALL RESULT ==========
    print("\n" + "="*40)
    if x_all_pass and y_all_pass:
        print("OVERALL: ALL SPECIFICATIONS MET! ✓")
    else:
        print("OVERALL: SOME SPECIFICATIONS FAILED ✗")
        print("\nRecommendations for tuning:")
        if not x_all_pass:
            print("  X-Position needs adjustment")
        if not y_all_pass:
            print("  Y-Position needs adjustment")
    print("="*40 + "\n")

if __name__ == "__main__":
    try:
        verify_specs()
    except Exception as e:
        print(f"\nERROR: {e}")
        print("\nMake sure scipy is installed:")
        print("  pip install scipy")
        sys.exit(1)

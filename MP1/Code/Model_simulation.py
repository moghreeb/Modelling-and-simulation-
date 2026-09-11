import numpy as np
import matplotlib.pyplot as plt


# ================== FUNCTION DEFINITION ==================
def robot_simulation(t, T, L1, L2, q1_start, q1_end, q2_start, q2_end):
    # Automatic Code/Model Check: Time validation
    if np.any(t < 0) or np.any(t > T):
        raise ValueError("Simulation stopped: Time vector (t) is outside the valid range [0, T].")

    # Normalized time and cubic profile
    u = t / T
    s = 3 * u ** 2 - 2 * u ** 3

    # Joint angles
    q1 = q1_start + (q1_end - q1_start) * s
    q2 = q2_start + (q2_end - q2_start) * s

    # Forward Kinematics (X, Y)
    x = L1 * np.cos(q1) + L2 * np.cos(q1 + q2)
    y = L1 * np.sin(q1) + L2 * np.sin(q1 + q2)

    # Analytical derivative for velocity
    ds_dt = (6 * u - 6 * u ** 2) * (1 / T)
    q1_dot = (q1_end - q1_start) * ds_dt
    q2_dot = (q2_end - q2_start) * ds_dt

    return x, y, q1, q2, q1_dot, q2_dot


# ================== MAIN SCRIPT ==================
# 1. Define Student Specific Parameters (S = 17)
S = 17
L1 = 0.380 + 0.005 * S  # Link 1 length (m)
L2 = 0.300 + 0.004 * S  # Link 2 length (m)
q1_start = np.deg2rad(20 + S)
q1_end = np.deg2rad(75 - 0.5 * S)
q2_start = np.deg2rad(-55 + 0.8 * S)
q2_end = np.deg2rad(15 + 0.5 * S)
T_nominal = 2.40 + 0.04 * S  # Nominal cycle time (s)

T_cases = [0.8 * T_nominal, T_nominal, 1.2 * T_nominal]
labels = ['0.8T', '1.0T', '1.2T']
colors = ['r', 'b', 'k']

# 2. Setup Figures
# Figure 1: Spatial Trajectory
fig1, ax_path = plt.subplots(figsize=(6, 6))
ax_path.set_title('End-Effector Path (Spatial Trajectory)')
ax_path.set_xlabel('X Position (m)')
ax_path.set_ylabel('Y Position (m)')
ax_path.grid(True)

# Figure 2: Velocity Parameter Study
fig2, (ax_v1, ax_v2) = plt.subplots(1, 2, figsize=(12, 5))
fig2.suptitle('Velocity Parameter Study')
ax_v1.set_title('Joint 1 Velocity Comparison')
ax_v1.set_xlabel('Time (s)')
ax_v1.set_ylabel('Angular Velocity (rad/s)')
ax_v1.grid(True)

ax_v2.set_title('Joint 2 Velocity Comparison')
ax_v2.set_xlabel('Time (s)')
ax_v2.set_ylabel('Angular Velocity (rad/s)')
ax_v2.grid(True)

# Variables to store baseline data for Figure 3
baseline_data = {}

# 3. Run Simulations
for i, T in enumerate(T_cases):
    dt = 0.05
    # Ensure the exact end time T is included by using np.arange and appending T if necessary
    t = np.arange(0, T + dt, dt)
    if t[-1] > T:
        t[-1] = T

    x, y, q1, q2, q1_dot, q2_dot = robot_simulation(t, T, L1, L2, q1_start, q1_end, q2_start, q2_end)

    # Plot Spatial Path ONLY ONCE (since it doesn't change with time)
    if i == 1:  # Index 1 corresponds to 1.0T (Baseline)
        ax_path.plot(x, y, linewidth=2, color='b', label='Path')
        ax_path.plot(x[0], y[0], 'go', markersize=8, label='Start')
        ax_path.plot(x[-1], y[-1], 'rs', markersize=8, label='End')
        ax_path.legend(loc='best')

        # Save baseline data for Figure 3
        baseline_data = {'t': t, 'q1': q1, 'q2': q2, 'q1_dot': q1_dot, 'q2_dot': q2_dot}

    # Plot Velocities in Figure 2
    ax_v1.plot(t, q1_dot, color=colors[i], linewidth=1.5, label=labels[i])
    ax_v2.plot(t, q2_dot, color=colors[i], linewidth=1.5, label=labels[i])

ax_v1.legend(loc='best')
ax_v2.legend(loc='best')

# 4. Figure 3: Baseline Case (Angles and Velocities)
fig3, (ax_ang, ax_vel) = plt.subplots(1, 2, figsize=(12, 5))
fig3.suptitle('Baseline Case (1.0T)')

# Subplot 1: Baseline Joint Angles
ax_ang.set_title('Baseline Joint Angles')
ax_ang.set_xlabel('Time (s)')
ax_ang.set_ylabel('Angle (Degrees)')
ax_ang.plot(baseline_data['t'], np.rad2deg(baseline_data['q1']), linewidth=2, label='Joint 1 (q1)')
ax_ang.plot(baseline_data['t'], np.rad2deg(baseline_data['q2']), linewidth=2, label='Joint 2 (q2)')
ax_ang.grid(True)
ax_ang.legend(loc='best')

# Subplot 2: Baseline Joint Velocities
ax_vel.set_title('Baseline Joint Velocities')
ax_vel.set_xlabel('Time (s)')
ax_vel.set_ylabel('Angular Velocity (rad/s)')
ax_vel.plot(baseline_data['t'], baseline_data['q1_dot'], linewidth=2, label='Joint 1 Velocity')
ax_vel.plot(baseline_data['t'], baseline_data['q2_dot'], linewidth=2, label='Joint 2 Velocity')
ax_vel.grid(True)
ax_vel.legend(loc='best')

# Extract and print performance measures
print("--- Quantitative Performance Measures ---")
print(f"Nominal Case (1.0T = {T_nominal:.2f} s):")
print(f"1. Start Coordinates (X, Y): ({x[0]:.4f} m, {y[0]:.4f} m)")
print(f"2. End Coordinates (X, Y): ({x[-1]:.4f} m, {y[-1]:.4f} m)")
path_length = np.sum(np.sqrt(np.diff(x) ** 2 + np.diff(y) ** 2))
print(f"3. Path Length: {path_length:.4f} m\n")

# Display all plots
plt.tight_layout()
plt.show()
% Mini Project 1 - MR Track (Two-Link Robot Pick-Cycle Kinematic Simulation)
% Student Number: S = 17

clc; clear; close all;

%% 1. Define Student Specific Parameters
S = 17;
L1 = 0.380 + 0.005*S;       % Link 1 length (m)
L2 = 0.300 + 0.004*S;       % Link 2 length (m)
q1_start = deg2rad(20 + S);           
q1_end   = deg2rad(75 - 0.5*S);       
q2_start = deg2rad(-55 + 0.8*S);      
q2_end   = deg2rad(15 + 0.5*S);       
T_nominal = 2.40 + 0.04*S;  % Nominal cycle time (s)

T_cases = [0.8*T_nominal, T_nominal, 1.2*T_nominal];
labels = {'0.8T', '1.0T', '1.2T'};
colors = {'r', 'b', 'k'};

%% 2. Figure 1: End-Effector Path (Spatial Trajectory)
fig1 = figure('Name', 'Figure 1: Spatial Trajectory', 'Position', [100, 100, 500, 500]);
hold on; grid on;
title('End-Effector Path');
xlabel('X Position (m)'); ylabel('Y Position (m)');

%% 3. Figure 2: Velocity Parameter Study
fig2 = figure('Name', 'Figure 2: Velocity Parameter Study', 'Position', [650, 100, 900, 400]);

% Subplot for Joint 1
ax1 = subplot(1,2,1); hold on; grid on;
title('Joint 1 Velocity Comparison');
xlabel('Time (s)'); ylabel('Angular Velocity (rad/s)');

% Subplot for Joint 2
ax2 = subplot(1,2,2); hold on; grid on;
title('Joint 2 Velocity Comparison');
xlabel('Time (s)'); ylabel('Angular Velocity (rad/s)');

%% 4. Run Simulations
for i = 1:length(T_cases)
    T = T_cases(i);
    dt = 0.05; 
    t = 0:dt:T;
    
    [x, y, q1, q2, q1_dot, q2_dot] = robot_simulation(t, T, L1, L2, q1_start, q1_end, q2_start, q2_end);
    
    % Plot Spatial Path ONLY ONCE (since it doesn't change with time)
    if i == 2 
        figure(fig1);
        plot(x, y, 'LineWidth', 2, 'Color', 'b', 'DisplayName', 'Path');
        plot(x(1), y(1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'DisplayName', 'Start');
        plot(x(end), y(end), 'rs', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', 'End');
        legend('Location', 'best');
        
        % Save baseline data for Figure 3
        t_base = t; q1_base = q1; q2_base = q2;
        q1_dot_base = q1_dot; q2_dot_base = q2_dot;
    end
    
    % Plot Velocities in Figure 2
    figure(fig2);
    plot(ax1, t, q1_dot, 'Color', colors{i}, 'LineWidth', 1.5, 'DisplayName', labels{i});
    plot(ax2, t, q2_dot, 'Color', colors{i}, 'LineWidth', 1.5, 'DisplayName', labels{i});
end

figure(fig2);
legend(ax1, 'Location', 'best');
legend(ax2, 'Location', 'best');

%% 5. Figure 3: Baseline Case (Angles and Velocities)
fig3 = figure('Name', 'Figure 3: Baseline Case (1.0T)', 'Position', [200, 600, 900, 400]);

subplot(1,2,1); hold on; grid on;
title('Baseline Joint Angles');
xlabel('Time (s)'); ylabel('Angle (Degrees)');
plot(t_base, rad2deg(q1_base), 'LineWidth', 2, 'DisplayName', 'Joint 1 (q1)');
plot(t_base, rad2deg(q2_base), 'LineWidth', 2, 'DisplayName', 'Joint 2 (q2)');
legend('Location', 'best');

subplot(1,2,2); hold on; grid on;
title('Baseline Joint Velocities');
xlabel('Time (s)'); ylabel('Angular Velocity (rad/s)');
plot(t_base, q1_dot_base, 'LineWidth', 2, 'DisplayName', 'Joint 1 Velocity');
plot(t_base, q2_dot_base, 'LineWidth', 2, 'DisplayName', 'Joint 2 Velocity');
legend('Location', 'best');

%% ================== FUNCTION DEFINITION ==================
function [x, y, q1, q2, q1_dot, q2_dot] = robot_simulation(t, T, L1, L2, q1_start, q1_end, q2_start, q2_end)
    if any(t < 0) || any(t > T)
        error('Simulation stopped: Time vector (t) is outside the valid range [0, T].');
    end
    
    u = t / T;
    s = 3*u.^2 - 2*u.^3;
    
    q1 = q1_start + (q1_end - q1_start) * s;
    q2 = q2_start + (q2_end - q2_start) * s;
    
    x = L1 * cos(q1) + L2 * cos(q1 + q2);
    y = L1 * sin(q1) + L2 * sin(q1 + q2);
    
    ds_dt = (6*u - 6*u.^2) * (1/T);
    q1_dot = (q1_end - q1_start) * ds_dt;
    q2_dot = (q2_end - q2_start) * ds_dt;
end
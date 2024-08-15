% DC Motor Parameters
J = 2.28e-7;    % kg.m^2
B = 5.09e-5;    % Nm.s/rad
L = 0.411e-3;   % H
R = 6.23;       % Ohm
K_t = 17.8e-3;  % Nm/A
K_e = 17.8e-3;  % V/(rad/s)

% State-space matrices including position
A = [0 1 0;
    0 -B/J K_t/J;
    0 -K_e/L -R/L];
B = [0; 0; 1/L];
C = [1 0 0];  % Control position (theta) as output
D = 0;

% Create state-space model
sys = ss(A, B, C, D);

%Ref Gain
K_r = 22;

% LQR Design
Q = diag([5000, 100, 1]);  % State cost matrix [position; velocity; current]
R = 10;  % Input cost

% Compute LQR Optimal Gains
K = lqr(A, B, Q, R); 

% Closed-loop system
sys_loop = ss((A - B*K), B*K_r, C, D);

% Simulation parameters
t = 0:0.00001:2;  % Time vector
r = ones(size(t)) * 200 * pi/180;  % Step input 

% Simulate
[y, t, x] = lsim(sys_loop, r, t);

% Calculate power consumption
I = x(:,3);  % Current
V = -K*x';   % Voltage also control input
P = V .* I'; % Power consumption over time

figure;
subplot(4,1,1);
plot(t, y);
xlabel("Time (s)");
ylabel("Position (rad)");
title("Position Response");

subplot(4,1,2);
plot(t, x(:,2));
xlabel("Time (s)");
ylabel("Input Voltage (V)");
title("Input Voltage");

subplot(4,1,3);
plot(t, P);
xlabel("Time (s)");
ylabel("Power (W)");
title("Power Consumption");

subplot(4,1,4);
plot(t, I);
xlabel("Time (s)");
ylabel("Current (A)");
title("Current");

% Settling time
settling_time = stepinfo(y, t, "SettlingTimeThreshold", 0.02);
settling_time_value = settling_time.SettlingTime;

% Rise time
rise_time = settling_time.RiseTime;

% Steady-state error
steady_state_error = abs(r(end) - y(end));

% Overshoot
overshoot = settling_time.Overshoot;

% Display the results
fprintf("Settling Time: %.4f s\n", settling_time_value);
fprintf("Rise Time: %.4f s\n", rise_time);
fprintf("Steady-State Error: %.4f rad\n", steady_state_error);
fprintf("Overshoot: %.2f%%\n", overshoot);
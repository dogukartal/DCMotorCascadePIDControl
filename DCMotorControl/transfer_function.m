% DC Motor Parameters
J = 2.28 * 10^(-7); %kg.m^2
B = 5.09 * 10^(-5); %Nm.s/rad
L = 0.411 * 10^(-3); %H
R = 6.23; %Ohm
K_t = 17.8 * 10^(-3); %Nm/A
K_e = 17.8 * 10^(-3); %V/(rad/s)

% DC Motor Tf
numerator = K_t;
denominator = [L*J, (L*B + R*J), R*B + K_t*K_e];
G_p = tf(numerator, denominator);

% PID parameters for inner loop
Kp_vel = 0.61;  
Ki_vel = 0.67;  
Kd_vel = 0.000005;  
G_in = pid(Kp_vel, Ki_vel, Kd_vel); %Transfer Function of inner PID

% PID parameters for outer loop 
Kp_pos = 135;
Ki_pos = 0.2694;
Kd_pos = 0.0134;
G_out = pid(Kp_pos, Ki_pos, Kd_pos); %Transfer Function of outer PID

% Integrator
integrator = tf(1, [1,0]);

% System Transfer Function
inner_loop = feedback(G_in * G_p, 1);
sys = feedback(G_out * inner_loop * integrator , 1);

% Step Plot
figure;
step(sys);
grid on

% Bode Plot
figure;
bode(sys);
grid on

[mag, phase, freq] = bode(sys);
mag_dB = 20*log10(squeeze(mag));

% Find frequencies where the magnitude is within -3 dB of the peak
peak_mag_dB = max(mag_dB);
bandwidth_indices = find(mag_dB >= (peak_mag_dB - 3));
bandwidth_range = freq(bandwidth_indices);
% Convert to Hz
max_bandwidth = max(bandwidth_range) / (2*pi);
clear

%Simulation Parameters
dt = 0.00001; %s

%DC Motor Model Parameters
V_in = 24;
J = 2.28 * 10^(-7); %kg.m^2
B = 5.09 * 10^(-5); %Nm.s/rad
L = 0.411 * 10^(-3); %H
R = 6.23; %Ohm
K_t = 17.8 * 10^(-3); %Nm/A
K_e = 17.8 * 10^(-3); %V/(rad/s)

%PID Gains
Kp_pos = 140;
Ki_pos = 0.2694;
Kd_pos = 0.0134;
Kp_vel = 0.6;
Ki_vel = 0.67;
Kd_vel = 0.000005;

%Encoder Parameter
cpr = 360000;

%Step Function
amp = 200;
start_time = 1;

%Chirp Signal
start_freq = 1;
end_freq = 20;
time_range = 200;

%Signal Switch: 1 for Step, 0 for Chirp
signal_switch = 1;
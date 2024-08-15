function [theta, omega, current] = DCmotor(V_in, T_load, J, B, L, R, K_t, K_e, dt)
    % V_in: Input Voltage (V)
    % T_load: Load torque (Nm) = 0
    % J: Moment of Inertia (kg.m^2) {2.28 x 10^(-7)}
    % B: Viscous Damping Coefficient (Nm.s/rad) {5.09 x 10^(-5)}
    % L: Inductance (H) {0.411 x 10^(-3)}
    % R: Resistance (Ohm) {6.23}
    % K_t: Torque Constant (Nm/A) {17.8 x 10^(-3)}
    % K_e: Back EMF Constant (V/(rad/s)) {0.0178}
    % dt: Time step (s)

    persistent prev_theta prev_omega prev_current
    
    % Parameter Initialization
    if isempty(prev_theta)
        prev_theta = 0;
        prev_omega = 0;
        prev_current = 0;
    end

    %Electrical Equation
    dI_dt = (V_in - K_e * prev_omega - R * prev_current) / L;
    current = prev_current + dI_dt * dt;

    %Mechanical Equation
    domega_dt = (K_t * current - B * prev_omega - T_load) / J;
    omega = prev_omega + domega_dt * dt;

    %Update States
    theta = prev_theta + prev_omega * dt;
 
    prev_theta = theta;
    prev_omega = omega;
    prev_current = current;
end
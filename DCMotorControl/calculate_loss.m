%PID Gains
Kp_pos = 140;
Ki_pos = 0.2694;
Kd_pos = 0.43;
Kp_vel = 0.6;
Ki_vel = 0.67;
Kd_vel = 0.000005;

sim_out = sim('SimModel', 'StopTime', '10');
y = sim_out.logsout{1}.Values.Data;
t = sim_out.get('tout');

info = stepinfo(y, t);
settling_time = info.SettlingTime; 
rise_time = info.RiseTime; 
steady_state_error = abs(1 - y(end)); 
overshoot = info.Overshoot;
ITAE = trapz(t, t .* abs(1 - y)); %Integral of Time-Weighted Absolute Error
ISE = trapz(t, (1 - y).^2); % Integral of Squared Error

w_settling = 1000;
w_rise = 2000;
w_steady_state = 100;
w_overshoot = 50;
w_itae = 100;
w_ise = 100;

cost = w_settling * (settling_time - 1)^2 + ...
       w_rise * (rise_time - 1)^2 + ...
       w_steady_state * steady_state_error^2 + ...
       w_overshoot * overshoot^2 + ...
       w_ise * ISE + ...
       w_itae * ITAE;

fprintf('Settling Time: %.4f, Rise Time: %.4f, SS Error: %.4f, Overshoot: %.4f, ISE: %.4f, ITAE: %.4f\n',  settling_time, rise_time, steady_state_error, overshoot, ISE, ITAE);
fprintf("Loss: %.4f ", cost);


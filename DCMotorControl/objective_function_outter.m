function cost = objective_function_outter(x)

    % Overwrite PID parameters for loss calculation
    assignin('base', 'Kp_pos', x(1));
    assignin('base', 'Ki_pos', x(2));
    assignin('base', 'Kd_pos', x(3));

    try
        % Run Simulink model
        warning('off', 'all');
        sim_out = sim('SimModel');
        warning('on', 'all');

        % Extract logged data from simulation
        y = sim_out.logsout{1}.Values.Data; % Theta (DC Motor Position)
        t = sim_out.get('tout');

        % Check for instability with three conditions
        if any(isnan(y)) || any(isinf(y)) || max(abs(y)) > 1e6
            cost = 1e5;  % High cost for unstable systems
        else
            info = stepinfo(y, t);
            settling_time = info.SettlingTime; % Settiling Time Penalty
            rise_time = info.RiseTime; % Rise Time Penalty
            steady_state_error = abs(1 - y(end)); % Steady State Error Penalty
            overshoot = info.Overshoot; % Overshoot Penalty
            ITAE = trapz(t, t .* abs(1 - y)); %Integral of Time-Weighted Absolute Error
            ISE = trapz(t, (1 - y).^2); % Integral of Squared Error

            % Define weights for each criterion
            w_settling = 10000;
            w_rise = 10000;
            w_steady_state = 100;
            w_overshoot = 1000;
            w_itae = 10;
            w_ise = 10;

            % Calculate cost (lower is better)
            cost = w_settling * (settling_time - 1) + ...
                   w_rise * (rise_time - 1) + ...
                   w_steady_state * steady_state_error + ...
                   w_overshoot * overshoot + ...
                   w_ise * ISE + ...
                   w_itae * ITAE;

            % Add a small penalty for large gains to prevent excessive values
            cost = cost + 0.01 * sum(x.^2);
        end
    catch
        cost = 1e10;  % High cost for simulation errors
    end
    %Output
    fprintf('Parameters: %.4f %.4f %.4f, Cost: %.4f\n', x(1), x(2), x(3), cost);
    fprintf("\n");
end
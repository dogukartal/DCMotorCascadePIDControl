function [Kp_pos, Ki_pos, Kd_pos] = optimize_cascade_pid_outter()

    lb = [0, 0, 0 ];  % Lower bounds
    ub = [500, 10, 1];  % Upper bounds
    
    % Initial Guesses
    initiate = [115, 0.3, 0.05];

    % Options for fmincon
    options = optimoptions("fmincon", "Display", "iter", "MaxIterations", 1000, "Algorithm", "interior-point");
    
    % Run the optimization
    [parameters, ~] = fmincon(@objective_function_outter, initiate, [], [], [], [], lb, ub, [], options);
    
    % Optimized parameters
    Kp_pos = parameters(1); 
    Ki_pos = parameters(2); 
    Kd_pos = parameters(3);

    fprintf("Position PID: Kp = %.4f, Ki = %.4f, Kd = %.4f\n", Kp_pos, Ki_pos, Kd_pos);
end
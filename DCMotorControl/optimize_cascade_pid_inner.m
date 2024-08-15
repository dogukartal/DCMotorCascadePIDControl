function [Kp_vel, Ki_vel, Kd_vel] = optimize_cascade_pid_inner()
    %Optimization of Inner PID Coefficients

    lb = [0, 0, 0 ];  % Lower bounds
    ub = [500, 10, 1];  % Upper bounds

    % Initial Guesses
    initiate = [0.5, 0.5, 0.000005];
    
    % Options for fmincon
    options = optimoptions("fmincon", "Display", "iter", "MaxIterations", 1000, "Algorithm", "interior-point");
    
    % Run the optimization
    [parameters, ~] = fmincon(@objective_function_inner, initiate, [], [], [], [], lb, ub, [], options);
    
    % Optimized parameters
    Kp_vel = parameters(1); 
    Ki_vel = parameters(2); 
    Kd_vel = parameters(3);
    
    fprintf("Velocity PID: Kp = %.4f, Ki = %.4f, Kd = %.4f\n", Kp_vel, Ki_vel, Kd_vel);

end
function u = PID(error, Kp, Ki, Kd, dt)  
    persistent prev_error error_sum 
    
    %Initiating parameters
    if isempty(prev_error)
        prev_error = 0;
        error_sum = 0;
    end
    
    %Proportional
    P = Kp * error;
    
    %Integral
    error_sum = error_sum + error * dt;
    I = Ki * error_sum;
    
    %Derivative
    D = Kd * (error - prev_error) / dt;
    
    %Control Signal
    u = P + I + D; 
    
    %Update of Previous Error
    prev_error = error;
end
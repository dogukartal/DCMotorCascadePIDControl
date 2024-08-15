function [step] = generate_step(amp, start_time, current_time)
    if current_time >= start_time %Signal Time Condition
        step = amp;
    else
        step = 0;
    end
end
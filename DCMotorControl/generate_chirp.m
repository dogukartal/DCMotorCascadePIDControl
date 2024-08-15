function [chirp] = generate_chirp(start_freq, end_freq, time_range, start_time, current_time)
    if current_time >= start_time && current_time <= time_range % Signal Time Condition
        k = (end_freq - start_freq) / time_range;
        chirp = sin(2 * pi * (start_freq * current_time + 0.5 * k * current_time^2));
    else
        chirp = 0;
    end
end
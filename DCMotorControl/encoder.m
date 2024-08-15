function [encoder_output] = encoder(theta, cpr)
    count = round(theta * (cpr / (2 * pi)));
    encoder_output = count * (2 * pi / cpr);
end
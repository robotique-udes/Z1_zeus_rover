function [rpm] = rads2rpm(radPerSeconds)
    rpm = (60 / (2 * pi)) * radPerSeconds;
end


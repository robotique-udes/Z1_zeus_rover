function [unpadded_motor_current, time] = unpad( motor )
%   Takes a row vector, removes all zeros at the beginning of the vector,
%       and returns the remaining elements of the vector

motor_current = motor.status.OutputCurrent;
index = find(motor.status.OutputCurrent ~= 0, 1, 'first');

inv = flip(motor_current);
index_inv = find(inv ~= 0, 1, 'first');
unpadded_motor_current = motor_current(index : end-index_inv);

normalized_timestamp = motor.status.rosbagTimestamp - motor.status.rosbagTimestamp(1);
percent_time = normalized_timestamp/normalized_timestamp(end);
time_seconds = percent_time * (motor.status.secs(end) - motor.status.secs(1));
total_time = time_seconds(end) - time_seconds(1);


time = time_seconds(index : end-index_inv);
end
function [RESULTS] = get_results(directory)

path = pwd + "/raw_data/" + directory + "/" + directory;
RESULTS.name = directory;
RESULTS.motor.position = importPosition(path + "_current_position.csv");
RESULTS.motor.status = importStatus(path +  "_status.csv");
RESULTS.motor.percent = importMotorPercent(path + "_motor_percent.csv");

[RESULTS.current, RESULTS.time] = unpad(RESULTS.motor);

RESULTS.average_value = mean(RESULTS.current);
RESULTS.maximum_value = max(RESULTS.current);
RESULTS.energy_mAh = (RESULTS.average_value * RESULTS.time(end)) / 3.6;
RESULTS.averaged_current = movmean(RESULTS.current, 50);
RESULTS.total_time = RESULTS.time(end);
RESULTS.original_freq = 1/(RESULTS.total_time / length(RESULTS.time));

end


function [R] = computeNominalTorqueCase(C, V)
    
    R.gearbox_ratio = C.gearBoxRatio;
    R.wheel_quantity = V.wheelN;
    R.wheel_diameter = V.wheelD;
    R.motor_quantity = V.motorsN;

    R.motion_resistances = computeEntireVehicleMotionResistance(C, V);
    R.total_force_to_withdraw = 1.2 * R.motion_resistances.totalMotionResistance;

    R.torque_needed_axle = (R.total_force_to_withdraw * R.wheel_diameter) /  R.motor_quantity; 
    R.torque_needed_motor = (R.torque_needed_axle / R.gearbox_ratio);

    [~, closestIndex] = min(abs(C.CIM.TorqueNm - R.torque_needed_motor));
    R.realistic_motor_speed_needed_rpm = C.CIM.SpeedRPM(closestIndex);
    R.realistic_motor_speed_needed_rads = rpm2rads(R.realistic_motor_speed_needed_rpm);

    R.realistic_current_needed = C.CIM.CurrentA(closestIndex);
    R.realistic_motor_power_needed = R.realistic_current_needed * 12;
    R.realistic_linear_speed = ((R.realistic_motor_speed_needed_rpm / R.gearbox_ratio) * V.circonferenceInMeters) / 60;

    R.current_needed_motor = R.realistic_current_needed;
        
end


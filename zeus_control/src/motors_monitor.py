#!/usr/bin/env python 

# -*- coding: utf-8 -*-

# Created on Jul 12 2021
# @author: Simon Chamorro       simon.chamorro@usherbrooke.ca


"""
@package zeus_control

------------------------------------

ROS Node to monitor information about the motors and give warnings

"""

import rospy
import numpy as np
from std_msgs.msg import Int32, Bool, Float32
from ros_talon.msg import Status
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure


class MotorsMonitorNode():
    def __init__(self):
        '''
        Node class to monitor rover motors
        '''
        rospy.init_node('motors_monitor', anonymous=False)
        rospy.on_shutdown(self.on_shutdown)
        self.motors_voltage = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
        self.motors_temperature = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
        self.motors_current = np.array([0.0,0.0,0.0,0.0,0.0,0.0])

        # Init publishers
        self.temperature_pub = rospy.Publisher('/zeus_monitor/avg_temperature', Float32, queue_size=10)
        self.voltage_pub = rospy.Publisher('/zeus_monitor/avg_voltage', Float32, queue_size=10)
        self.current_pub = rospy.Publisher('/zeus_monitor/avg_current', Float32, queue_size=10)
        self.warning_pub = rospy.Publisher('/zeus_monitor/motor_warning', Bool, queue_size=10)
        
        # Subscribers 
        self.m1_current_sub = rospy.Subscriber('/ros_talon1/status', Status, self.motor_callback, 0)
        self.m1_current_sub = rospy.Subscriber('/ros_talon2/status', Status, self.motor_callback, 1)
        self.m1_current_sub = rospy.Subscriber('/ros_talon3/status', Status, self.motor_callback, 2)
        self.m1_current_sub = rospy.Subscriber('/ros_talon4/status', Status, self.motor_callback, 3)
        self.m1_current_sub = rospy.Subscriber('/ros_talon5/status', Status, self.motor_callback, 4)
        self.m1_current_sub = rospy.Subscriber('/ros_talon6/status', Status, self.motor_callback, 5)

        # Initialize configurable params
        # Create a DynamicDynamicReconfigure Server
        self.ddr = DDynamicReconfigure("motors_monitor")

        # Add variables to ddr(name, description, default value, min, max, edit_method)        
        self.ddr.add_variable("temperature_max", "float", 40.0, 0.0, 100.0)
        self.ddr.add_variable("temperature_min", "float", 10.0, -10.0, 100.0)
        self.ddr.add_variable("voltage_max", "float", 24.0, 0.0, 50.0)
        self.ddr.add_variable("voltage_min", "float", 13.0, 0.0, 24.0)
        self.ddr.add_variable("current_max", "float", 20.0, 10.0, 30.0)
        self.ddr.add_variable("current_min", "float", -1.0, -10.0, 100.0)
        
        # Start Server
        self.ddr.start(self.dynamic_reconfigure_callback)
        rospy.sleep(1)

        # Init command loop 
        rospy.Timer(rospy.Duration(1.0/5.0), self.monitor_data_callback)


    def dynamic_reconfigure_callback(self, config, level):
        '''
        Updates parameters value when changed by the user.
        ----------
        Parameters
        ----------
        config: dict
            Keys are param names and values are param values
        level: Unused
        -------
        Returns
        -------
        config: dict
            Keys are param names and values are param values
        '''
        # Update variables
        var_names = self.ddr.get_variable_names()
        for var_name in var_names:
            self.__dict__[var_name] = config[var_name]
        return config


    def motor_callback(self, msg, motor_nb):
        '''
        Callback when motor status is rceived
        ----------
        Parameters
        ----------
        msg: Status
            Status message
        motor_nb: Int
            motor ID 
        '''
        self.motors_temperature[motor_nb] = msg.Temperature
        self.motors_current[motor_nb] = msg.OutputCurrent
        self.motors_voltage[motor_nb] = msg.BusVoltage


    def monitor_data_callback(self, evt):
        '''
        Timer function that checks data and publishes aggregated status
        '''
        error = ''
        warning = False
        temperature_avg = self.motors_temperature.mean()
        voltage_avg = self.motors_voltage.mean()
        current_avg = self.motors_current.mean() 

        # Check temperatures
        if not self.inside_bounds(temperature_avg, self.temperature_min, self.temperature_max):
            warning = True
            error += 'Temperature is outside desired range\n'
        else:
            inside, idx = self.inside_bounds_array(self.motors_temperature, self.temperature_min, self.temperature_max)
            if not inside:
                warning = True
                error += 'Temperature is outside desired range for motor ' + str(idx) + ' \n'

        # Check voltage
        if not self.inside_bounds(voltage_avg, self.voltage_min, self.voltage_max):
            warning = True
            error += 'Voltage is outside desired range\n'
        else:
            inside, idx = self.inside_bounds_array(self.motors_voltage, self.voltage_min, self.voltage_max)
            if not inside:
                warning = True
                error += 'Voltage is outside desired range for motor ' + str(idx) + ' \n'

        # Check current
        if not self.inside_bounds(current_avg, self.current_min, self.current_max):
            warning = True
            error += 'Current is outside desired range\n'
        else:
            inside, idx = self.inside_bounds_array(self.motors_current, self.current_min, self.current_max)
            if not inside:
                warning = True
                error += 'Current is outside desired range for motor ' + str(idx) + ' \n'

        # Messages can be monitored through rqt topic monitor 
        # and warnings can be monitored with rqt_console
        self.temperature_pub.publish(temperature_avg)
        self.voltage_pub.publish(voltage_avg)
        self.current_pub.publish(current_avg)
        self.warning_pub.publish(warning)
        if warning:
            rospy.logwarn('[ZEUS_STATUS] ' + error)


    def inside_bounds(self, data, min_val, max_val):
        '''
        Checks if value is inside bounds
        ----------
        Parameters
        ----------
        data: Float
        min_val: Float
        max_val: Float
        ----------
        Returns
        ----------
        is_inside_bounds: Bool
        '''
        return False if data > max_val or data < min_val else True


    def inside_bounds_array(self, data, min_val, max_val):
        '''
        Checks if values of array are inside bounds
        ----------
        Parameters
        ----------
        data: np.Array
        min_val: Float
        max_val: Float
        ----------
        Returns
        ----------
        is_inside_bounds: Bool
        idx: Int
            Which value is not inside the bounds
        '''
        for idx, val in enumerate(data):
            if val > max_val or val < min_val:
                return False, idx
        return True, 0


    def on_shutdown(self):
        '''
        Shutdown
        '''
        pass



if __name__ == '__main__':
    try:
        node = MotorsMonitorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


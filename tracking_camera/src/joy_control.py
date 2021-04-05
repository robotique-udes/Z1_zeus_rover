#!/usr/bin/env python 

# -*- coding: utf-8 -*-

# Created on Apr 5 2021
# @author: Simon Chamorro       simon.chamorro@usherbrooke.ca


"""
@package tracking_camera

------------------------------------

ROS Node control pan-tilt system with xbox controller

"""

import rospy
import numpy as np
from constants import *
from std_msgs.msg import Int32
from sensor_msgs.msg import Joy
from motor_control import MotorControl
from dynamixel_workbench_msgs.msg import *
from dynamixel_workbench_msgs.srv import *
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure



class PanTiltControl():
    def __init__(self):
        '''
        Node class to teleoperate pan-tilt system
        '''
        rospy.init_node('pan_tilt_control', anonymous=False)
        rospy.on_shutdown(self.on_shutdown)
        self.motor_control = MotorControl()
        self.cmd = [0, 0]
        self.motor_state = [0, 0]
        self.motor_sub = rospy.Subscriber("/dynamixel_workbench/dynamixel_state", DynamixelStateList, self.update_motors)

        # Initialize configurable params
        # Create a DynamicDynamicReconfigure Server
        self.ddr = DDynamicReconfigure("pan_tilt")

        # Add variables to ddr(name, description, default value, min, max, edit_method)        
        self.ddr.add_variable("gain", "float: max speed gain", 1.0, 0.0, 10.0)
        
        # Start Server
        self.ddr.start(self.dynamic_reconfigure_callback)
        rospy.sleep(1)

        # Subscribe to joystick
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)


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


    def update_motors(self, state):
        '''
        Updates motor class internal state
        '''
        for motor in state.dynamixel_state:
            if motor.name == "yaw":
                yaw = motor.present_position 
            elif motor.name == "pitch":
                pitch = motor.present_position
        self.motor_state = [yaw, pitch] 
        self.motor_control.update_motors([yaw, pitch])


    def go_home(self):
        '''
        Send motors to home position
        '''
        self.motor_control.go_home()


    def enable_motors(self, enable):
        '''
        Enable motors
        '''
        self.motor_control.enable_motors(enable)


    def move_axis(self, axis, motor_id, value):
        '''
        Move one axis
        '''
        self.motor_control.move_axis(axis, motor_id, value)


    def limit_pos(self, pos, axis):
        '''
        Limits position
        '''
        if axis == 0:
            limits = YAW
        elif axis == 1:
            limits = PITCH 

        pos = min(pos, limits[1])
        pos = max(pos, limits[0])
            
        return pos


    def joy_callback(self, msg):
        '''
        Callback from joy message
        ----------
        Parameters
        ----------
        msg: Joy
            Joy message
        '''
        # Go home
        if msg.buttons[3]:
            self.go_home()

        else:
            # Pan 
            pan_cmd = self.limit_pos(self.motor_state[0] + msg.axes[6]*self.gain, 0)

            # Tilt
            tilt_cmd = self.limit_pos(self.motor_state[1] + msg.axes[7]*self.gain, 1)

            self.cmd = [pan_cmd, tilt_cmd]
            self.send_cmd()


    def send_cmd(self):
        '''
        Publishes commands
        '''
        self.move_axis(0, 3, self.cmd[0])    
        self.move_axis(1, 2, self.cmd[1])  


    def on_shutdown(self):
        pass



if __name__ == '__main__':
    try:
        node = PanTiltControl()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


#!/usr/bin/env python 

# -*- coding: utf-8 -*-

# Created on Apr 5 2021
# @author: Simon Chamorro       simon.chamorro@usherbrooke.ca


"""
@package zeus_control

------------------------------------

ROS Node control pan-tilt system with xbox controller

"""

import rospy
import numpy as np
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import Joy
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure



class PanTiltControl():
    def __init__(self):
        '''
        Node class to teleoperate pan-tilt system
        '''
        rospy.init_node('pan_tilt_control', anonymous=False)
        rospy.on_shutdown(self.on_shutdown)
        self.motor_pos = [90, 90]
        self.motor_min = [1, 85]
        self.motor_max = [180, 180]

        # Initialize configurable params
        # Create a DynamicDynamicReconfigure Server
        self.ddr = DDynamicReconfigure("pan_tilt")

        # Add variables to ddr(name, description, default value, min, max, edit_method)        
        self.ddr.add_variable("gain", "float: max speed gain", 5, 1, 20)
        
        # Start Server
        self.ddr.start(self.dynamic_reconfigure_callback)
        rospy.sleep(1)

        # Init publishers
        self.pan_tilt_pub = rospy.Publisher("/zeus_control/pantilt", Int16MultiArray, queue_size=10)

        # Subscribe to joystick
        self.go_home()
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


    def go_home(self):
        '''
        Send motors to home position
        '''
        self.cmd = [90, 90]
        self.send_cmd()


    def limit_pos(self, cmd):
        '''
        Limits position
        '''
        cmd[0] = min(cmd[0], self.motor_max[0])
        cmd[0] = max(cmd[0], self.motor_min[0])

        cmd[1] = min(cmd[1], self.motor_max[1])
        cmd[1] = max(cmd[1], self.motor_min[1])
            
        return cmd


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
            # Commands 
            pan_cmd = self.cmd[0] + msg.axes[6]*self.gain
            tilt_cmd = self.cmd[1] - msg.axes[7]*self.gain

            self.cmd = self.limit_pos([pan_cmd, tilt_cmd])
            self.send_cmd()


    def send_cmd(self):
        '''
        Publishes commands
        '''
        msg = Int16MultiArray()
        msg.data.append(self.cmd[0])
        msg.data.append(self.cmd[1])
        self.pan_tilt_pub.publish(msg)


    def on_shutdown(self):
        pass



if __name__ == '__main__':
    try:
        node = PanTiltControl()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


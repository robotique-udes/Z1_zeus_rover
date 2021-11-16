#!/usr/bin/env python 

# -*- coding: utf-8 -*-

# Created on Oct 8 2021
# @author: Simon Chamorro       simon.chamorro@usherbrooke.ca


"""
@package zeus_control

------------------------------------

ROS Node to go from a motor angular desired speed to current %

"""

import rospy
import numpy as np
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure


class MotorPIDNode():
    def __init__(self):
        '''
        Node class to control motor commands
        '''
        rospy.init_node('motor_pid', anonymous=False)
        rospy.on_shutdown(self.on_shutdown)

        self.motor_nb = rospy.get_param('motor_nb', 1)
        self.output_topic = "/ros_talon" + str(self.motor_nb) + "/motor_percent"
        self.input_cmd_topic = "/ros_talon" + str(self.motor_nb) + "/motor_speed"
        self.input_enc_topic = "/ros_talon" + str(self.motor_nb) + "/current_position"

        # Parameters
        self.gear_ratio = 40
        self.wheel_radius = 0.28
        self.rad2enc = 4096*2*np.pi

        # Initial state
        self.pos = None
        self.speed = 0.0
        self.error = 0.0
        self.total_error = 0.0
        self.last_error = 0.0
        self.last_error_time = rospy.get_rostime().to_sec()

        # Initialize configurable params
        # Create a DynamicDynamicReconfigure Server
        self.ddr = DDynamicReconfigure(rospy.get_name())

        self.ddr.add_variable("p_gain", "float", 1, 0.01, 10.0)
        self.ddr.add_variable("i_gain", "float", 1, 0.01, 10.0)
        self.ddr.add_variable("d_gain", "float", 1, 0.01, 10.0)

        # Initialize configurable params
        # Create a DynamicDynamicReconfigure Server
        self.ddr = DDynamicReconfigure("twist2cmd")

        # Start Server
        self.ddr.start(self.dynamic_reconfigure_callback)
        rospy.sleep(1)

        # Init publisher
        self.motor_pub = rospy.Publisher(self.output_topic, Int32, queue_size=10)

        # Subscribers
        self.motor_cmd_sub = rospy.Subscriber(self.input_cmd_topic, Float32, self.cmd_callback)
        self.motor_enc_sub = rospy.Subscriber(self.input_enc_topic, Float32, self.enc_callback)


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


    def cmd_callback(self, msg):
        '''
        Callback when cmd is received
        ----------
        Parameters
        ----------
        msg: Float32
            Float message (angular motor speed)
        '''
        pass


    def enc_callback(self, msg):
        '''
        Callback when encoder msg is received
        ----------
        Parameters
        ----------
        msg: Float32
            Float message (angular motor speed)
        '''
        if self.pos == None:
            self.speed = 0.0

        else:

        pass


    def send_cmd(self):
        '''
        Publishes commands
        '''
        self.motor_pub.publish(self.cmd)


    def on_shutdown(self):
        '''
        Set commands to 0 at shutdown
        '''
        self.cmd = 0
        self.send_cmd()



if __name__ == '__main__':
    try:
        node = MotorPIDNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


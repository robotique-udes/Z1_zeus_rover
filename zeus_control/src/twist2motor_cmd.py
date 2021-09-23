#!/usr/bin/env python 

# -*- coding: utf-8 -*-

# Created on Feb 8 2021
# @author: Simon Chamorro       simon.chamorro@usherbrooke.ca


"""
@package zeus_control

------------------------------------

ROS Node to go from a Twist to commands for the 6 robot motors

"""

import rospy
import numpy as np
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure


class LowLevelControlNode():
    def __init__(self):
        '''
        Node class to teleoperate arm joint by joint
        '''
        rospy.init_node('twist2cmd', anonymous=False)
        rospy.on_shutdown(self.on_shutdown)
        self.l_cmd_center = 0
        self.r_cmd_center = 0
        self.l_cmd_outer = 0
        self.r_cmd_outer = 0
        self.last_cmd = [0, 0, 0, 0, 0, 0]
        self.last_cmd_time = [rospy.get_rostime().to_sec()]*6
        self.last_twist_received = 0.0

        # Init publishers
        self.m1_pub = rospy.Publisher('/ros_talon1/motor_percent', Int32, queue_size=10)
        self.m2_pub = rospy.Publisher('/ros_talon2/motor_percent', Int32, queue_size=10)
        self.m3_pub = rospy.Publisher('/ros_talon3/motor_percent', Int32, queue_size=10)
        self.m4_pub = rospy.Publisher('/ros_talon4/motor_percent', Int32, queue_size=10)
        self.m5_pub = rospy.Publisher('/ros_talon5/motor_percent', Int32, queue_size=10)
        self.m6_pub = rospy.Publisher('/ros_talon6/motor_percent', Int32, queue_size=10)

        # Subscribe to joystick
        self.twist_sub = rospy.Subscriber('zeus_control/cmd_vel', Twist, self.twist_callback)

        # Initialize configurable params
        # Create a DynamicDynamicReconfigure Server
        self.ddr = DDynamicReconfigure("twist2cmd")

        # Add variables to ddr(name, description, default value, min, max, edit_method)        
        self.ddr.add_variable("stop_timeout", "float", 0.5, 0.01, 2.0)
        self.ddr.add_variable("linear_gain", "float", 1, 0, 2)
        self.ddr.add_variable("angular_gain", "float", -4, -8, 8)
        self.ddr.add_variable("max_motor_cmd", "int", 100, 1, 100)
        self.ddr.add_variable("max_accel", "float", 100, 1, 300)
        self.ddr.add_variable("outer_wheels_rotation_gain", "float", 1.1, 1.0, 1.5)

        # Individual gains for wheels
        self.ddr.add_variable("wheel_1_gain", "float", 1.0, -2.0, 2.0)
        self.ddr.add_variable("wheel_2_gain", "float", 1.0, -2.0, 2.0)
        self.ddr.add_variable("wheel_3_gain", "float", 1.0, -2.0, 2.0)
        self.ddr.add_variable("wheel_4_gain", "float", 1.0, -2.0, 2.0)
        self.ddr.add_variable("wheel_5_gain", "float", 1.0, -2.0, 2.0)
        self.ddr.add_variable("wheel_6_gain", "float", 1.0, -2.0, 2.0)

        # Start Server
        self.ddr.start(self.dynamic_reconfigure_callback)
        rospy.sleep(1)

        # Init command loop 
        rospy.Timer(rospy.Duration(1.0/50), self.send_cmd_callback)


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


    def twist_callback(self, msg):
        '''
        Callback when twist is rceived
        ----------
        Parameters
        ----------
        msg: Twist
            Twist message
        '''
        # Send same command to all motors of same side
        l_cmd_center = -msg.linear.x*100*self.linear_gain - msg.angular.z*30*self.angular_gain
        r_cmd_center = msg.linear.x*100*self.linear_gain - msg.angular.z*30*self.angular_gain
        l_cmd_outer = -msg.linear.x*100*self.linear_gain - msg.angular.z*30*self.angular_gain*self.outer_wheels_rotation_gain
        r_cmd_outer = msg.linear.x*100*self.linear_gain - msg.angular.z*30*self.angular_gain*self.outer_wheels_rotation_gain

        # Limit speed 
        self.l_cmd_center = self.limit_speed(l_cmd_center, max_val=self.max_motor_cmd)
        self.r_cmd_center = self.limit_speed(r_cmd_center, max_val=self.max_motor_cmd)
        self.l_cmd_outer = self.limit_speed(l_cmd_outer, max_val=self.max_motor_cmd)
        self.r_cmd_outer = self.limit_speed(r_cmd_outer, max_val=self.max_motor_cmd)

        self.last_twist_received = rospy.get_rostime().to_sec()


    def limit_speed(self, cmd, max_val=60):
        '''
        Limits command at a certain value
        '''
        if cmd > 0:
            cmd = min(cmd, max_val)
        else:
            cmd = max(cmd, -max_val)
        return cmd


    def limit_accel(self, cmd, idx):
        '''
        Limits command at a certain acceleration based on past commands
        '''
        # Compute acceleration
        out_cmd = 0
        last_t = self.last_cmd_time[idx]
        last_cmd = self.last_cmd[idx]
        delta_cmd = cmd - last_cmd
        current_t = rospy.get_rostime().to_sec()
        delta_t = current_t - last_t
        accel = delta_cmd / delta_t

        # Limit command
        if abs(accel) <= self.max_accel:
            out_cmd = cmd

        else:
            out_cmd = last_cmd + self.max_accel*delta_t*np.sign(delta_cmd)

        # Store values 
        self.last_cmd[idx] = out_cmd
        self.last_cmd_time[idx] = current_t
        return out_cmd


    def send_cmd_callback(self, evt):
        '''
        Send commands to motors timer
        '''
        self.send_cmd()



    def send_cmd(self):
        '''
        Publishes commands
        '''
        if self.last_twist_received + self.stop_timeout > rospy.get_rostime().to_sec():
            # Left wheels
            self.m1_pub.publish(self.limit_accel(self.limit_speed(self.l_cmd_outer*self.wheel_1_gain, max_val=100), 0))
            self.m3_pub.publish(self.limit_accel(self.limit_speed(self.l_cmd_center*self.wheel_3_gain, max_val=100), 2))
            self.m5_pub.publish(self.limit_accel(self.limit_speed(self.l_cmd_outer*self.wheel_5_gain, max_val=100), 4))
            
            # # Right wheels
            self.m2_pub.publish(self.limit_accel(self.limit_speed(self.r_cmd_outer*self.wheel_2_gain, max_val=100), 1))
            self.m4_pub.publish(self.limit_accel(self.limit_speed(self.r_cmd_center*self.wheel_4_gain, max_val=100), 3))
            self.m6_pub.publish(self.limit_accel(self.limit_speed(self.r_cmd_outer*self.wheel_6_gain, max_val=100), 5))
        else:
            # Left wheels
            self.m1_pub.publish(self.limit_accel(self.limit_speed(0, max_val=100), 0))
            self.m3_pub.publish(self.limit_accel(self.limit_speed(0, max_val=100), 2))
            self.m5_pub.publish(self.limit_accel(self.limit_speed(0, max_val=100), 4))
            
            # Right wheels
            self.m2_pub.publish(self.limit_accel(self.limit_speed(0, max_val=100), 1))
            self.m4_pub.publish(self.limit_accel(self.limit_speed(0, max_val=100), 3))
            self.m6_pub.publish(self.limit_accel(self.limit_speed(0, max_val=100), 5))


    def on_shutdown(self):
        '''
        Set commands to 0 at shutdown
        '''
        self.l_cmd_center = 0
        self.r_cmd_center = 0
        self.l_cmd_outer = 0
        self.r_cmd_outer = 0
        self.send_cmd()



if __name__ == '__main__':
    try:
        node = LowLevelControlNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


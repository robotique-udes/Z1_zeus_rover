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
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist


class LowLevelControlNode():
    def __init__(self):
        '''
        Node class to teleoperate arm joint by joint
        '''
        rospy.init_node('twist2cmd', anonymous=False)
        rospy.on_shutdown(self.on_shutdown)
        self.cmd = 0

        # Init publishers
        self.m1_pub = rospy.Publisher('/ros_talon1/motor_percent', Int32, queue_size=10)
        self.m2_pub = rospy.Publisher('/ros_talon2/motor_percent', Int32, queue_size=10)
        self.m3_pub = rospy.Publisher('/ros_talon3/motor_percent', Int32, queue_size=10)
        self.m4_pub = rospy.Publisher('/ros_talon4/motor_percent', Int32, queue_size=10)
        self.m5_pub = rospy.Publisher('/ros_talon5/motor_percent', Int32, queue_size=10)
        self.m6_pub = rospy.Publisher('/ros_talon6/motor_percent', Int32, queue_size=10)

        # Init command loop 
        rospy.Timer(rospy.Duration(1.0/50), self.send_cmd_callback)

        # Subscribe to joystick
        self.twist_sub = rospy.Subscriber('zeus_control/cmd_vel', Twist, self.twist_callback)


    def twist_callback(self, msg):
        '''
        Callback when twist is rceived
        ----------
        Parameters
        ----------
        msg: Twist
            Twist message
        '''
        # For now, only use linear speed
        cmd = msg.linear.x * 75

        # Limit speed at 40%
        if cmd > 0:
            cmd = min(cmd, 40)
        else:
            cmd = max(cmd, -40)
        
        self.cmd = cmd


    def send_cmd_callback(self, evt):
        '''
        Send commands to motors timer
        '''
        self.send_cmd()


    def send_cmd(self):
        '''
        Publishes commands
        '''
        self.m1_pub.publish(self.cmd)
        self.m2_pub.publish(self.cmd)
        self.m3_pub.publish(self.cmd)
        self.m4_pub.publish(self.cmd)
        self.m5_pub.publish(self.cmd)
        self.m6_pub.publish(self.cmd)


    def on_shutdown(self):
        '''
        Set commands to 0 at shutdown
        '''
        self.cmd = 0
        self.send_cmd()



if __name__ == '__main__':
    try:
        node = LowLevelControlNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


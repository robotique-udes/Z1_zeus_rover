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
        self.l_cmd = 0
        self.r_cmd = 0

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
        # Send same command to all motors of same side
        l_cmd = msg.linear.x*75 + msg.angular.z*30
        r_cmd = msg.linear.x*75 - msg.angular.z*30

        # Limit speed at 60%
        self.l_cmd = self.limit_speed(l_cmd, max_val=50)
        self.r_cmd = self.limit_speed(r_cmd, max_val=50)


    def limit_speed(self, cmd, max_val=60):
        '''
        Limits command at a certain value
        '''
        if cmd > 0:
            cmd = min(cmd, max_val)
        else:
            cmd = max(cmd, -max_val)
        return cmd


    def send_cmd_callback(self, evt):
        '''
        Send commands to motors timer
        '''
        self.send_cmd()


    def send_cmd(self):
        '''
        Publishes commands
        '''
        # Left wheels
        self.m1_pub.publish(self.l_cmd)
        self.m3_pub.publish(self.l_cmd)
        self.m5_pub.publish(self.l_cmd)
        
        # Right wheels
        self.m2_pub.publish(self.r_cmd)
        self.m4_pub.publish(self.r_cmd)
        self.m6_pub.publish(self.r_cmd)


    def on_shutdown(self):
        '''
        Set commands to 0 at shutdown
        '''
        self.l_cmd = 0
        self.r_cmd = 0
        self.send_cmd()



if __name__ == '__main__':
    try:
        node = LowLevelControlNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


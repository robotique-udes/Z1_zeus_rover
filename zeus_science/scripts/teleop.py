#!/usr/bin/env python 

# -*- coding: utf-8 -*-

# Created on April 9 2021
# @author: Simon Chamorro       simon.chamorro@usherbrooke.ca
#          Santiago Moya        santiago.moya@usherbrooke.ca


"""
@package zeus_science

------------------------------------

ROS Node to teleoperate the science module

"""

import time
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16


class ScienceTeleopNode():
    def __init__(self):
        '''
        Node class to teleoperate the science module
        '''
        rospy.init_node('teleop_science', anonymous=False)
        rospy.on_shutdown(self.on_shutdown)
        self.cmd = Int16()
        self.last_change = time.time()

        # Subscribe to joy and publish command
        self.joy_sub = rospy.Subscriber('/joy_science', Joy, self.joy_callback)
        self.cmd_pub = rospy.Publisher('/zeus_science/command', Int16 , queue_size=10)

    def joy_callback(self, msg):
        '''
        Callback from joystick
        ----------
        Parameters
        ----------
        msg: Joy
            Message from joystick
        '''
        cmd = Int16()
        if msg.buttons[0]:
            self.cmd.data = 1
        elif msg.buttons[1]:
            self.cmd.data = 2
        elif msg.buttons[2]:
            self.cmd.data = 3
        elif msg.buttons[3]:
            self.cmd.data = 4
        elif msg.buttons[4]:
            self.cmd.data = 5
        elif msg.buttons[5]:
            self.cmd.data = 6
        elif msg.buttons[6]:
            self.cmd.data = 7
        elif msg.buttons[7]:
            self.cmd.data = 8
        elif msg.buttons[8]:
            self.cmd.data = 9
        elif msg.buttons[9]:
            self.cmd.data = 10
        else:
            self.cmd.data = 0

        self.send_cmd()
        
    def print_cmd(self):
        rospy.loginfo("Sent command : %s", str(self.cmd.data))

    def send_cmd(self):
        '''
        Publishes commands
        '''
        if time.time() - self.last_change > 0.3:
            self.cmd_pub.publish(self.cmd)
            self.last_change = time.time()

    def on_shutdown(self):
        '''
        Set commands to 0 at shutdown
        '''
        self.cmd.data = 0
        self.send_cmd
        
if __name__ == '__main__':
    try:
        node = ScienceTeleopNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


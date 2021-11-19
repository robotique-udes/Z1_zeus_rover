#!/usr/bin/env python 

# -*- coding: utf-8 -*-

# Created on Sept 30 2021
# @author: Samuel Hovington       samuel.hovington@usherbrooke.ca


"""
@package zeus_control

------------------------------------

ROS Node to monitor information about the rover state and control light tower

"""

import rospy
import rosnode
import numpy as np
from ros_talon.msg import Status
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Bool, Int32MultiArray, Float32
from multimaster_msgs_fkie.srv import DiscoverMasters
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure


LIGHT_MODES = {'OFF': 0,
               'ON': 2,
               'FLASH': 1}

LIGHT_COLOR = { 'GREEN' :   0,
                'YELLOW' :  1,
                'RED' :     2}

class ZeusMonitorNode():
    def __init__(self):
        '''
        Node class to monitor rover motors
        '''
        rospy.init_node('zeus_monitor', anonymous=False)
        rospy.on_shutdown(self.on_shutdown)
        self.light_msg = Int32MultiArray()
        self.light_msg.data = [0, 0, 0]
        self.twist = Twist()

        # Init publishers
        self.light_pub = rospy.Publisher('/zeus_monitor/light_control', Int32MultiArray, queue_size=10)

        # Subscribers 
        self.twist_sub = rospy.Subscriber('/zeus_control/cmd_vel', Twist, self.twist_callback)

        # Init command loop 
        rospy.Timer(rospy.Duration(1.0/10.0), self.monitor_data_callback)

    def twist_callback(self, msg):
        '''
        Callback when motor status is rceived
        ----------
        Parameters
        ----------
        msg: Twist
            Twist command msg
        '''
        self.twist = msg


    def monitor_data_callback(self, evt):
        '''
        Timer function that checks data and publishes aggregated status
        '''

        # Check status of rover 
        moving = self.rover_is_moving()
        
        if moving:
            self.light_msg.data[LIGHT_COLOR["YELLOW"]] = LIGHT_MODES['FLASH']
            self.light_msg.data[LIGHT_COLOR["RED"]] = LIGHT_MODES['OFF']
        
        else:
            self.light_msg.data[LIGHT_COLOR["YELLOW"]] = LIGHT_MODES['OFF']
            self.light_msg.data[LIGHT_COLOR["RED"]] = LIGHT_MODES['ON']

        # Publish messages
        self.light_pub.publish(self.light_msg)

        self.twist.linear.x = 0.0

    

    def rover_is_moving(self):
        '''
        Check if rover is moving
        ----------
        Return
        ----------
        moving: Bool
        '''
        print(self.twist.linear.x)
        moving_linear = self.twist.linear.x > 0.01 or self.twist.linear.x < -0.01
        moving_angular = self.twist.angular.z > 0.01 or self.twist.angular.z < -0.01
        return moving_angular or moving_linear



    def on_shutdown(self):
        '''
        Shutdown
        '''
        pass



if __name__ == '__main__':
    try:
        node = ZeusMonitorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


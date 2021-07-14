#!/usr/bin/env python 

# -*- coding: utf-8 -*-

# Created on Mar 12 2021
# @author: Samuel       samuel.hovington@usherbrooke.ca


"""
@package zeus_control

------------------------------------

ROS Node to publish odom from robot's 6 wheels position

"""
from math import sin, cos, pi

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu


class HeadingPublisher():
    def __init__(self):
        '''
        Node class to publish odom from robot's 6 wheels position
        '''
        rospy.init_node('heading_publisher', anonymous=False)
        rospy.on_shutdown(self.on_shutdown)


        # Subscriber
        ## subscribe to motor's encoder
        rospy.Subscriber("/imu/data", Imu, self.imuCallback)

        # Publisher
        self.headingPub = rospy.Publisher("/heading", PoseStamped, queue_size=10)
        # self.odomBroadcaster = TransformBroadcaster()


    def imuCallback(self, msg):
        heading = PoseStamped()
        heading.header = msg.header
        heading.header.frame_id = "/map"
        heading.pose.orientation = msg.orientation
        self.headingPub.publish(heading)



    def on_shutdown(self):
        pass



if __name__ == '__main__':
    try:
        node = HeadingPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
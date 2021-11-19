#!/usr/bin/env python 

# -*- coding: utf-8 -*-

# Created on Aug 9 2021
# @author: Simon Chamorro       simon.chamorro@usherbrooke.ca


"""
@package zeus_control

------------------------------------

ROS Node to save trajectory from published GPS coords

"""

import os
import csv
import rospy
import numpy as np
from sensor_msgs.msg import NavSatFix


TRAJECTORY_PATH = 'trajectory.csv'

class TrajectoryNode():
    def __init__(self):
        '''
        Node class to save gps trajectory
        '''
        rospy.init_node('save_trajectory', anonymous=False)
        rospy.on_shutdown(self.on_shutdown)

        # Open the file and create the csv writer
        self.file = open(TRAJECTORY_PATH, 'w')
        self.writer = csv.writer(self.file)

        # Init subscriber
        self.gps_sub = rospy.Subscriber('/nav/fix', NavSatFix, self.fix_callback)


    def to_string(self, msg):
        point_str = str(msg.latitude) + ', ' 
                    + str(msg.longitude) + ', '
                    + str(msg.altitude) + ', '
                    + str(msg.header.stamp.sec)
        return point_str


    def fix_callback(self, msg):
        # Write a row to the csv file
        point_str = self.to_string(msg)
        self.writer.writerow(point_str)


    def on_shutdown(self):
        # Close the file
        self.file.close()


if __name__ == '__main__':
    try:
        node = TrajectoryNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


#!/usr/bin/env python 

# -*- coding: utf-8 -*-

# Created on Aug 9 2021
# @author: Simon Chamorro       simon.chamorro@usherbrooke.ca


"""
@package zeus_control

------------------------------------

ROS Node to save geotagged dosimeter data published GPS coords + dosimeter readings

"""

import os
import csv
import rospy
import numpy as np
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32MultiArray


SAVE_PATH = 'dosimeter.csv'

class DosimeterNode():
    def __init__(self):
        '''
        Node class to save gps trajectory + dosimeter data
        '''
        rospy.init_node('save_dosimeter', anonymous=False)
        rospy.on_shutdown(self.on_shutdown)

        # Open the file and create the csv writer
        self.dosimeter_msg = Float32MultiArray()
        self.file = open(SAVE_PATH, 'w')
        self.writer = csv.writer(self.file)

        # Init subscribers
        self.gps_sub = rospy.Subscriber('/nav/fix', NavSatFix, self.fix_callback)
        self.dosimeter_sub = rospy.Subscriber('/dosimeter/data', Float32MultiArray, self.dosimeter_callback)


    def to_string(self, gps_msg, dosimeter_msg):
        data_str = str(gps_msg.latitude) + ', ' 
                    + str(gps_msg.longitude) + ', '
                    + str(gps_msg.altitude) + ', '
                    + str(gps_msg.header.stamp.sec) + ', '
                    + str(dosimeter_msg.data[0]) + ', '
                    + str(dosimeter_msg.data[1])
        return data_str


    def dosimeter_callback(self, msg):
        # Save data
        self.dosimeter_msg = msg


    def fix_callback(self, msg):
        # Write a row to the csv file
        point_str = self.to_string(msg, self.dosimeter_msg)
        self.writer.writerow(point_str)


    def on_shutdown(self):
        # Close the file
        self.file.close()


if __name__ == '__main__':
    try:
        node = DosimeterNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


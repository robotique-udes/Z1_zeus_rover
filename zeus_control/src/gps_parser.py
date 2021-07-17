#!/usr/bin/env python 

# -*- coding: utf-8 -*-

# Created on Jul 17 2021
# @author: Simon Chamorro       simon.chamorro@usherbrooke.ca


"""
@package zeus_control

------------------------------------

ROS Node to parse csv file and publish GPS coords

"""

import os
import csv
import rospy
import numpy as np
from sensor_msgs.msg import NavSatFix


GPS_PATH = 'gps_coords.csv'

class GPSParserNode():
    def __init__(self):
        '''
        Node class to publish gps points
        '''
        rospy.init_node('gps_parser', anonymous=False)
        rospy.on_shutdown(self.on_shutdown)
        self.points = []
        self.parse_csv(GPS_PATH)

        # Init publishers
        self.gps_pub = rospy.Publisher('/waypoints', NavSatFix, queue_size=10)

        # Init command loop 
        rospy.Timer(rospy.Duration(1.0/5.0), self.gps_pub_callback)


    def parse_csv(self, file_name):
        '''
        Parse GPS points from csv
        ----------
        Parameters
        ----------
        file_name: string
            Relative path to file
        '''
        with open(file_name) as csvfile:
            reader = csv.reader(csvfile, delimiter=' ', quotechar='|')
            for row in reader:
                if len(row) == 2:
                    self.points.append((float(row[0]), float(row[1])))


    def gps_pub_callback(self, evt):
        '''
        Callback from GPS publishing timer
        '''
        if len(self.points) > 0:
            msg = NavSatFix()
            point = self.points.pop()
            msg.latitude = point[0]
            msg.longitude = point[1]
            self.gps_pub.publish(msg)
        else:
            rospy.signal_shutdown('Done parsing and publishing')


    def on_shutdown(self):
        pass



if __name__ == '__main__':
    try:
        node = GPSParserNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


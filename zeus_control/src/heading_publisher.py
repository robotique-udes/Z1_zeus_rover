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
from sensor_msgs.msg import Imu, NavSatFix
from gps_common.msg import GPSFix
import tf

class HeadingPublisher():
    def __init__(self):
        '''
        Node class to publish odom from robot's 6 wheels position
        '''
        rospy.init_node('heading_publisher', anonymous=False)
        rospy.on_shutdown(self.on_shutdown)


        # Subscriber
        ## subscribe to motor's encoder
        rospy.Subscriber("/imu/data_out", Imu, self.imuCallback)
        rospy.Subscriber("/fix", NavSatFix, self.gpsCallback)

        # Publisher
        self.gpsPub = rospy.Publisher("/fix_out", GPSFix, queue_size=10)

        rospy.Timer(rospy.Duration(1.0/5.0), self.timerCallback)

        self.gps_in = NavSatFix()
        self.imu = Imu()
        self.gps_out = GPSFix()


    def imuCallback(self, msg):
        self.imu.header = msg.header
        self.imu.orientation = msg.orientation

    def gpsCallback(self, msg):
        self.gps_in.header = msg.header
        self.gps_in.status = msg.status
        self.gps_in.latitude = msg.latitude
        self.gps_in.longitude = msg.longitude
        self.gps_in.altitude = msg.altitude
        self.gps_in.position_covariance = msg.position_covariance
        self.gps_in.position_covariance_type = msg.position_covariance_type

    def timerCallback(self, evt):
        self.gps_out.header = self.gps_in.header
        self.gps_out.header.stamp = rospy.Time.now()
        self.gps_out.latitude = 51.4675932
        self.gps_out.longitude = -112.7060198
        # self.gps_out.latitude = self.gps_in.latitude
        # self.gps_out.longitude = self.gps_in.longitude
        self.gps_out.altitude = self.gps_in.altitude
        self.gps_out.roll, self.gps_out.pitch, self.gps_out.track  = self.quat2yaw(self.imu.orientation)
        # self.dip = self.gps_out.track
        self.gpsPub.publish(self.gps_out)

    def quat2yaw(self, quat):
        quaternion = (
        quat.x,
        quat.y,
        quat.z,
        quat.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]*180/3.14159
        return roll, pitch, yaw

    def on_shutdown(self):
        pass



if __name__ == '__main__':
    try:
        node = HeadingPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
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
from std_msgs.msg import Float64MultiArray, Float32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf.broadcaster import TransformBroadcaster


class OdomPublisher():
    def __init__(self):
        '''
        Node class to publish odom from robot's 6 wheels position
        '''
        rospy.init_node('odom_publisher', anonymous=False)
        rospy.on_shutdown(self.on_shutdown)

        self.rate = 10.0  # the rate at which to publish the transform
        self.base_width = 0.6 # The wheel base width in meters
        self.ticks_meter = (6045*40)/(3.14159265359*0.2)  # The number of wheel encoder ticks per meter of travel
        
        self.base_frame_id = 'base_link' # the name of the base frame of the robot
        self.odom_frame_id = 'odom' # the name of the odometry reference frame

        self.encoder_min = 1000000000000 #6045 ticks/turn
        self.encoder_max = -1000000000000
        self.encoder_low_wrap = (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min
        self.encoder_high_wrap = (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min
        

        # Init command loop 
        rospy.Timer(rospy.Duration(1.0/self.rate), self.update)

        # Subscriber
        ## subscribe to motor's encoder
        rospy.Subscriber("/ros_talon1/current_position", Float32, self.lfwheelCallback)
        rospy.Subscriber("/ros_talon2/current_position", Float32, self.rfwheelCallback)
        rospy.Subscriber("/ros_talon5/current_position", Float32, self.lbwheelCallback)
        rospy.Subscriber("/ros_talon6/current_position", Float32, self.rbwheelCallback)

        # Publisher
        self.odomPub = rospy.Publisher("odom", Odometry, queue_size=10)
        # self.odomBroadcaster = TransformBroadcaster()

        # internal data
        self.enc_left = None        # wheel encoder readings
        self.enc_right = None
        self.left = 0               # actual values coming back from robot
        self.right = 0
        self.lmult = 0
        self.rmult = 0
        self.prev_lencoder = 0
        self.prev_rencoder = 0
        self.x = 0                  # position in xy plane 
        self.y = 0
        self.th = 0
        self.dx = 0                 # speeds in x/rotation
        self.dr = 0
        self.elapsed = 1.0/self.rate
        self.raw_encoder = [0, 0, 0, 0, 0, 0] # leftFront, leftMiddle, leftBack, rightFront, rightMiddle, rightBack

    
    def update(self, evt):
        self.lwheels()
        self.rwheels()
        # calculate odometry
        if self.enc_left == None:
            d_left = 0
            d_right = 0
        else:
            d_left = (self.left - self.enc_left) / self.ticks_meter
            d_right = (self.right - self.enc_right) / self.ticks_meter
        self.enc_left = self.left
        self.enc_right = self.right
        
        # distance traveled is the average of the two wheels 
        d = ( d_left + d_right ) / 2
        # this approximation works (in radians) for small angles
        th = ( d_right - d_left ) / self.base_width
        # calculate velocities
        self.dx = d / self.elapsed
        self.dr = th / self.elapsed
        
            
        if (d != 0):
            # calculate distance traveled in x and y
            x = cos( th ) * d
            y = -sin( th ) * d
            # calculate the final position of the robot
            self.x = self.x + ( cos( self.th ) * x - sin( self.th ) * y )
            self.y = self.y + ( sin( self.th ) * x + cos( self.th ) * y )
        if( th != 0):
            self.th = self.th + th
            
        self.publish_odom()

    def publish_odom(self):
        # publish the odom information
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = sin( self.th / 2 )
        quaternion.w = cos( self.th / 2 )
        
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = self.odom_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0
        odom.pose.pose.orientation = quaternion
        odom.child_frame_id = self.base_frame_id
        odom.twist.twist.linear.x = self.dx
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = self.dr
        self.odomPub.publish(odom)

    def lfwheelCallback(self, msg):
        self.raw_encoder[0] = msg.data

    def lmwheelCallback(self, msg):
        self.raw_encoder[1] = msg.data

    def lbwheelCallback(self, msg):
        self.raw_encoder[1] = msg.data

    def rfwheelCallback(self, msg):
        self.raw_encoder[3] = -msg.data

    def rmwheelCallback(self, msg):
        self.raw_encoder[4] = -msg.data

    def rbwheelCallback(self, msg):
        self.raw_encoder[4] = -msg.data

    def lwheels(self):
        print(self.raw_encoder)
        enc = sum(self.raw_encoder[0:2]) / 2.0

        if (enc < self.encoder_low_wrap and self.prev_lencoder > self.encoder_high_wrap):
            self.lmult = self.lmult + 1
            
        if (enc > self.encoder_high_wrap and self.prev_lencoder < self.encoder_low_wrap):
            self.lmult = self.lmult - 1
            
        self.left = 1.0 * (enc + self.lmult * (self.encoder_max - self.encoder_min)) 
        self.prev_lencoder = enc
        
    def rwheels(self):
        enc = sum(self.raw_encoder[3:5]) / 2

        if(enc < self.encoder_low_wrap and self.prev_rencoder > self.encoder_high_wrap):
            self.rmult = self.rmult + 1
        
        if(enc > self.encoder_high_wrap and self.prev_rencoder < self.encoder_low_wrap):
            self.rmult = self.rmult - 1
            
        self.right = 1.0 * (enc + self.rmult * (self.encoder_max - self.encoder_min))
        self.prev_rencoder = enc



    def on_shutdown(self):
        pass



if __name__ == '__main__':
    try:
        node = OdomPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
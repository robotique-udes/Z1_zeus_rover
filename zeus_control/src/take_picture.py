#!/usr/bin/env python 

# -*- coding: utf-8 -*-

# Created on Jul 30 2021
# @author: Simon Chamorro       simon.chamorro@usherbrooke.ca


"""
@package zeus_control

------------------------------------

ROS Node to save image from topic

"""

import os
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import NavSatFix, Image
from cv_bridge import CvBridge, CvBridgeError


class PictureNode():
    def __init__(self):
        '''
        Node class to take picture
        '''
        rospy.init_node('picture_node', anonymous=False)
        rospy.on_shutdown(self.on_shutdown)
        self.bridge = CvBridge()

        # Get topics
        self.image_topic = rospy.get_param('/picture_node/image', '/image_raw')
        self.gps_topic = rospy.get_param('/picture_node/gps', '/fix')
        self.path = rospy.get_param('/picture_node/path', '~/zeus_pictures/')
        print('Subscribing to: ' + self.image_topic)
        print('Subscribing to: ' + self.gps_topic)


    def get_filename(self):
        return str(rospy.get_rostime().nsecs)


    def save_coords(self, filename):
        # Get coords
        try:
            coords = rospy.wait_for_message(self.gps_topic, NavSatFix, timeout=1.0)
            with open(filename + '.txt', 'w') as f:
                f.write(str(coords))
            print('GPS message saved: ' + filename)
        except:
            print('Timeout: No gps position received')


    def take_picture(self):
        # Get filename for picture
        filename = self.path + self.get_filename()

        # Save gps coords
        self.save_coords(filename)

        # Take picture and save
        try:
            image = rospy.wait_for_message(self.image_topic, Image, timeout=1.0)
            cv2_img = self.bridge.imgmsg_to_cv2(image, "bgr8")
            cv2.imwrite(filename + '.jpg', cv2_img)
            print('Image saved: ' + filename)
        except:
            print('Timeout: No image received')


    def on_shutdown(self):
        pass



if __name__ == '__main__':
    try:
        node = PictureNode()
        node.take_picture()
    except rospy.ROSInterruptException:
        pass


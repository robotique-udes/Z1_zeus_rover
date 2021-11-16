#!/usr/bin/env python 

# -*- coding: utf-8 -*-

# Created on Aug 8 2021
# @author: Simon Chamorro       simon.chamorro@usherbrooke.ca


"""
@package zeus_control

------------------------------------

ROS Node to scan image for aruco markers

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
        rospy.init_node('aruco_picture_node', anonymous=False)
        rospy.on_shutdown(self.on_shutdown)
        self.bridge = CvBridge()

        # Aruco detector
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

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


    def plot_aruco(self, img, all_corners, ids, rejected):
        # verify at least one Aruco marker was detected
        if len(all_corners) > 0:
            # flatten the aruco IDs list
            ids = ids.flatten()

            # loop over detected aruco markers
            for (corner, marker_id) in zip(all_corners, ids):
                # Extract the marker corners (which are always returned in
                # top-left, top-right, bottom-right, and bottom-left order)
                corners = corner.reshape((4, 2))
                (top_left, top_right, bottom_right, bottom_left) = corners
                
                # Convert each of the (x, y) coordinate pairs to integers
                top_right = (int(top_right[0]), int(top_right[1]))
                bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
                bottom_left = (int(bottom_left[0]), int(bottom_left[1]))
                top_left = (int(top_left[0]), int(top_left[1]))

                # Draw the bounding box of the aruco detection
                cv2.line(img, top_left, top_right, (0, 255, 0), 2)
                cv2.line(img, top_right, bottom_right, (0, 255, 0), 2)
                cv2.line(img, bottom_right, bottom_left, (0, 255, 0), 2)
                cv2.line(img, bottom_left, top_left, (0, 255, 0), 2)
                
                # Compute and draw the center (x, y) coordinates of the
                # aruco marker
                cX = int((top_left[0] + bottom_right[0]) / 2.0)
                cY = int((top_left[1] + bottom_right[1]) / 2.0)
                cv2.circle(img, (cX, cY), 4, (0, 0, 255), -1)
                
                # draw the aruco marker ID on the frame
                cv2.putText(img, str(marker_id),
                    (top_left[0], top_left[1] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)
        return img


    def take_picture(self):
        # Get filename for picture
        filename = self.path + self.get_filename()

        # Save gps coords
        self.save_coords(filename)

        # Take picture, scan for aruco markers and save
        try:
            image = rospy.wait_for_message(self.image_topic, Image, timeout=1.0)
            cv2_img = self.bridge.imgmsg_to_cv2(image, "bgr8")
            (corners, ids, rejected) = cv2.aruco.detectMarkers(cv2_img, self.aruco_dict, parameters=self.aruco_params)
            out_img = self.plot_aruco(cv2_img, corners, ids, rejected)
            cv2.imwrite(filename + '.jpg', out_img)
            print('Image saved: ' + filename)
            cv2.imshow("Aruco", out_img)
            key = cv2.waitKey(0)
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


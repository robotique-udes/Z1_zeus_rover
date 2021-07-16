#!/usr/bin/env python 

# -*- coding: utf-8 -*-

# Created on Jul 12 2021
# @author: Simon Chamorro       simon.chamorro@usherbrooke.ca


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


LIGHT_MODES = {'ERROR': 0,
               'READY': 1,
               'MOVING': 2}

ROVER_MOVEMENT_NODES = ['/joy_node',
                        '/teleop_twist_joy',
                        '/twist_mux',
                        '/twist2cmd',
                        '/socketcan_bridge_node']

# TODO
ARM_MOVEMENT_NODES = []

LOCAL_HOSTS = ['http://192.168.1.31:11611',
               'http://192.168.1.32:11611',
               'http://192.168.1.33:11611']


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
        self.buzz_pub = rospy.Publisher('/zeus_monitor/buzz', Bool, queue_size=10)

        # Subscribers 
        self.twist_sub = rospy.Subscriber('/zeus_control/cmd_vel', Twist, self.twist_callback)
        # TODO: Check actual encoders instead of command?
        # self.right_sub = rospy.Subscriber("/ros_talon2/current_position", Float32, self.wheel_callback)
        # self.left_sub = rospy.Subscriber("/ros_talon1/current_position", Float32, self.wheel_callback)
        # TODO: Check for arm movement
        # self.arm_sub = rospy.Subscriber('/zeus_arm/joint_positions', Twist, self.arm_callback)

        # Initialize configurable params
        # Create a DynamicDynamicReconfigure Server
        self.ddr = DDynamicReconfigure("zeus_monitor")

        # Add variables to ddr(name, description, default value, min, max, edit_method)        
        self.ddr.add_variable("monitor_frequence", "float", 1.0, 0.1, 50.0)
        
        # Start Server
        self.ddr.start(self.dynamic_reconfigure_callback)
        rospy.sleep(1)

        # Init command loop 
        rospy.Timer(rospy.Duration(1.0/self.monitor_frequence), self.monitor_data_callback)


    def dynamic_reconfigure_callback(self, config, level):
        '''
        Updates parameters value when changed by the user.
        ----------
        Parameters
        ----------
        config: dict
            Keys are param names and values are param values
        level: Unused
        -------
        Returns
        -------
        config: dict
            Keys are param names and values are param values
        '''
        # Update variables
        var_names = self.ddr.get_variable_names()
        for var_name in var_names:
            self.__dict__[var_name] = config[var_name]
        return config


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


    def wheel_callback(self, msg):
        '''
        Callback when motor encoder is received
        ----------
        Parameters
        ----------
        msg: Float32
            Wheel encoder value
        '''
        pass


    def monitor_data_callback(self, evt):
        '''
        Timer function that checks data and publishes aggregated status
        '''
        # Check if communication with base station works
        masters = self.check_comm()
        self.light_msg.data[0] = LIGHT_MODES['READY'] if len(masters) > 0 else LIGHT_MODES['ERROR']

        # Check status of rover 
        moving = self.rover_is_moving()
        ready = self.check_nodes(ROVER_MOVEMENT_NODES)
        
        if moving:
            should_buzz_rover = self.light_msg.data[1] == LIGHT_MODES['ERROR']
            self.light_msg.data[1] = LIGHT_MODES['MOVING']
        elif ready:
            should_buzz_rover = self.light_msg.data[1] == LIGHT_MODES['ERROR']
            self.light_msg.data[1] = LIGHT_MODES['READY']
        else:
            self.light_msg.data[1] = LIGHT_MODES['ERROR']

        # Check status of rover arm
        moving = self.arm_is_moving()
        ready = self.check_nodes(ARM_MOVEMENT_NODES)
        if moving:
            should_buzz_arm = self.light_msg.data[2] == LIGHT_MODES['ERROR']
            self.light_msg.data[2] = LIGHT_MODES['MOVING']
        elif ready:
            should_buzz_arm = self.light_msg.data[2] == LIGHT_MODES['ERROR']
            self.light_msg.data[2] = LIGHT_MODES['READY']
        else:
            self.light_msg.data[2] = LIGHT_MODES['ERROR']

        # Publish messages
        self.light_pub.publish(self.light_msg)
        self.buzz_pub.publish(should_buzz_arm or should_buzz_rover)
        print("publish")


    def check_comm(self):
        '''
        Check if there is a master connected through multimaster
        other than the 3 local jetsons
        ----------
        Return
        ----------
        masters: List
            Master uris excluding local machines
        '''
        service_name = '/master_discovery/list_masters'
        masters = []
        print("here")
        try:
            # rospy.wait_for_service(service_name, rospy.Duration(5))
            masters = []
            list_masters = rospy.ServiceProxy(service_name, DiscoverMasters)
            response = list_masters()
            for master in response.masters:
                if not master.monitoruri in LOCAL_HOSTS:
                    masters.append(master.monitoruri)

        except rospy.ServiceException as e:
           print("Service call failed: %s"%e)
        return masters


    def check_nodes(self, nodes):
        '''
        Check if nodes are alive
        ----------
        Parameters
        ----------
        msg: List
            Nodes to check
        ----------
        Return
        ----------
        alive: Bool
        '''
        nodes_alive = rosnode.get_node_names()
        return all(node in nodes_alive for node in nodes)


    def rover_is_moving(self):
        '''
        Check if rover is moving
        ----------
        Return
        ----------
        moving: Bool
        '''
        moving_linear = self.twist.linear.x < 0.01 and self.twist.linear.x > -0.01
        moving_angular = self.twist.angular.z < 0.01 and self.twist.angular.z > -0.01
        return moving_angular or moving_linear


    def arm_is_moving(self):
        '''
        Check if arm is moving
        ----------
        Return
        ----------
        moving: Bool
        '''
        # TODO
        return False


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


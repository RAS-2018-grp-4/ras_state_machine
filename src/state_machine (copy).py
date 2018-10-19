#!/usr/bin/env python

import numpy as np
import math
import matplotlib.pyplot as plt
import rospy
import std_msgs.msg
from std_msgs.msg import String
import geometry_msgs.msg
from nav_msgs.msg import Odometry
import itertools
import tf
import time
from arduino_servo_control.srv import *


class StateMachine:
    def __init__(self):
        #####################################################
        #               Initialize Publisher                #
        #####################################################
        rospy.init_node('state_machine_node', anonymous=True)
        rospy.wait_for_service('/arduino_servo_control/set_servo_angles')
        # publishers
        #pub_path_following_VEL = rospy.Publisher('/keyboard/vel', geometry_msgs.msg.Twist, queue_size=1)
        self.pub_test = rospy.Publisher('/hello', std_msgs.msg.String, queue_size=1)
        self.pub_target_pose = rospy.Publisher('/target_pose', geometry_msgs.msg.Pose, queue_size=1)


        self.rate = rospy.Rate(10)

        
        # initialize members

         # inital state
        self.state = "look_for_object"

        # flag states
        self.detect_object_done = False
        self.path_following_done = False
        #self.gripping_done = False
        #self.release_done = False
        self.gripper_flag = "close"

        # 
        self.target_position = [0.0, 0.0, 0.0]
        self.target_orientation = [0.0, 0.0, 0.0]

    #####################################################
    #             /robot_odom Callback          #
    #####################################################
    def flagCallback(self, msg):

        flag = msg.data
        if flag == "detect_object_done" and self.state == "look_for_object":
            self.detect_object_done = True
        elif flag == "path_following_done":
            self.path_following_done = True
        elif flag == "grip_done":
            pass
        elif flag == "release_done":
            pass


    def send_message(self, target_position, target_orientation):
        POSE = geometry_msgs.msg.Pose()
        POSE.position.x = target_position[0]
        POSE.position.y = target_position[1]
        POSE.position.z = 0
        (r, p, y, w) = tf.transformations.quaternion_from_euler(target_orientation[0], target_orientation[1], target_orientation[2])
        POSE.orientation.z = y
        POSE.orientation.w = w
        self.pub_target_pose.publish(POSE)

        print("message sent to path follower", POSE)

    def state_loop(self):
        try:
            grip = rospy.ServiceProxy('/arduino_servo_control/set_servo_angles', SetServoAngles)
            grip(0, 180)
        except rospy.ServiceException, e:
            print "Service call failed: %s"


        while not rospy.is_shutdown():
            print("is in state: " + self.state)

            if self.state == "look_for_object":
                #send_message(lin_vel, ang_vel*GAIN)

                if (self.detect_object_done):

                    # specify new target pose
                    self.target_position = [0.50, 0.0, 0.0]
                    self.target_orientation = [0.0, 0.0, 0.0]
                    self.state = "follow_path"
                    self.detect_object_done = False

            elif self.state == "follow_path":
                self.send_message(self.target_position, self.target_orientation)

                if self.path_following_done == True:
                    print("Path following done")

                    if self.gripper_flag == "close":
                        time.sleep(2)
                        self.state = "grip_object"
                        self.path_following_done = False
                    elif self.gripper_flag == "release":    
                        self.state = "release_object"
                        self.path_following_done = False

            elif self.state == "grip_object":
                print("Should now grip the object")                
                try:
                    grip = rospy.ServiceProxy('/arduino_servo_control/set_servo_angles', SetServoAngles)
                    grip(60, 120)
                    time.sleep(3)

                    # specify new target pose
                    self.target_position = [1.0, 0.0, 0.0]
                    self.target_orientation = [0.0, 0.0, 0.0]
                    self.state = "follow_path"
                    self.gripper_flag = "release"
                    self.path_following_done = False

                except rospy.ServiceException, e:
                    print "Service call failed: %s"

            elif self.state == "release_object":
                print("Should now release the object")                
                try:
                    grip = rospy.ServiceProxy('/arduino_servo_control/set_servo_angles', SetServoAngles)
                    grip(0, 180)
                    time.sleep(3)

                    # task is done
                    self.state = "stop"
 

                except rospy.ServiceException, e:
                    print "Service call failed: %s"

            elif self.state == "stop":
                print("Task done")  
            self.rate.sleep()

	


if __name__ == '__main__':
    print("state machine started")

    sm = StateMachine()


    # subscribers
    #rospy.Subscriber("/robot_odom", Odometry, odomCallback)
    rospy.Subscriber("/flag_done", String, sm.flagCallback)

    sm.state_loop()

    ##rospy.spin()
 


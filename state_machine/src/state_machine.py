#!/usr/bin/env python

import rospy
import smach
import smach_ros
import numpy as np
import math
import matplotlib.pyplot as plt
import rospy
import std_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
import itertools
import tf
import time
import threading
from arduino_servo_control.srv import *


FLAG_GRIP = False
FLAG_START = True
FLAG_DETECT_OBJECT = False
FLAG_FOLLOW_PATH = False
pub_TARGET_POSE = rospy.Publisher('/target_pose', geometry_msgs.msg.Pose, queue_size=1)
pub_RESET = rospy.Publisher('/odom_reset', std_msgs.msg.Bool, queue_size=1)
#####################################################
#                  Initialization                   #
#####################################################
class Initialization(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Initialization Done'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Initialization')
        grip = rospy.ServiceProxy('/arduino_servo_control/set_servo_angles', SetServoAngles)
        grip(0, 180)
        msg = std_msgs.msg.Bool()
        msg.data = True
        pub_RESET.publish(msg)
        return 'Initialization Done'


#####################################################
#                    Standby                       #
#####################################################
class Standby(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Receive Start Message'])

        
    def execute(self, userdata):
        global FLAG_START
        rospy.loginfo('Executing state Standby')
        while not FLAG_START:
            pass
        FLAG_START = False
        return 'Receive Start Message'



        
#####################################################
#                Look For Object                   #
#####################################################
class Look_For_Object(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Detected Object'])

    def execute(self, userdata):
        global FLAG_DETECT_OBJECT
        rospy.loginfo('Executing state Look_For_Object')
        while not FLAG_DETECT_OBJECT:
            pass
        FLAG_DETECT_OBJECT = False
        return 'Detected Object'


#####################################################
#                 Follow_Path                       #
#####################################################
class Follow_Path(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Reached Gripping Target','Reached Releasing Target'])

    def execute(self, userdata):
        global FLAG_FOLLOW_PATH, FLAG_GRIP
        rospy.loginfo('Executing state Follow_Path')
        if not FLAG_GRIP:
            target_position = [0.35, 0.0, 0.0]
            target_orientation = [0.0, 0.0, 0.0]
        else:
            target_position = [-0.15, 0.0, 0.0]
            target_orientation = [0.0, 0.0, 0.0]
        send_message(target_position,target_orientation)
        while not FLAG_FOLLOW_PATH:
            pass
        FLAG_FOLLOW_PATH = False
        if not FLAG_GRIP:
            return 'Reached Gripping Target'
        else:
            return 'Reached Releasing Target'

#####################################################
#                 Grip_Object                       #
#####################################################
class Grip_Object(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Gripped Object'])

    def execute(self, userdata):
        global FLAG_GRIP
        rospy.loginfo('Executing state Grip_Object')
        time.sleep(1)
        grip = rospy.ServiceProxy('/arduino_servo_control/set_servo_angles', SetServoAngles)
        grip(60, 120)
        time.sleep(1)
        FLAG_GRIP = True
        return 'Gripped Object'

#####################################################
#                Release_Object                     #
#####################################################
class Release_Object(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Released Object'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Release_Object')
        grip = rospy.ServiceProxy('/arduino_servo_control/set_servo_angles', SetServoAngles)
        grip(0, 180)
        time.sleep(1)
        return 'Released Object'



#####################################################
#                Send_Message                       #
#####################################################
def send_message(target_position, target_orientation):
    global pub_TARGET_POSE
    POSE = geometry_msgs.msg.Pose()
    POSE.position.x = target_position[0]
    POSE.position.y = target_position[1]
    POSE.position.z = 0
    (r, p, y, w) = tf.transformations.quaternion_from_euler(target_orientation[0], target_orientation[1], target_orientation[2])
    POSE.orientation.z = y
    POSE.orientation.w = w
    pub_TARGET_POSE.publish(POSE)

#####################################################
#                  Feedback_Start                   #
#####################################################
def start_callback(msg):
    global FLAG_START
    if msg.data == True:
        FLAG_START = True
    else :
        pass

#####################################################
#             /robot_odom Callback                  #
#####################################################
def flag_callback(msg):
    global FLAG_DETECT_OBJECT,FLAG_FOLLOW_PATH, FLAG_GRIP, FLAG_START
    flag = msg.data
    if flag == "detect_object_done" : 
        FLAG_DETECT_OBJECT = True
        #print("detect_object_done")
    elif flag == "path_following_done" :
        FLAG_FOLLOW_PATH = True
        #print("path_following_done")
    elif flag == "grip_done":
        pass
    elif flag == "release_done":
        pass

def obj_position_callback(msg):
        #print(msg.data) 
        pass

def main():
    rospy.init_node('state_machine_node')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['Stop'])
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    rospy.wait_for_service('/arduino_servo_control/set_servo_angles')
    rospy.Subscriber('/Start', std_msgs.msg.Bool, start_callback)
    rospy.Subscriber("/flag_done", String, flag_callback)
    rospy.Subscriber("/object_position", Float32MultiArray, obj_position_callback)

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Initialization', Initialization(), 
                               transitions={'Initialization Done':'Standby'})

        smach.StateMachine.add('Standby', Standby(), 
                               transitions={'Receive Start Message':'Look_For_Object'})

        smach.StateMachine.add('Look_For_Object', Look_For_Object(), 
                               transitions={'Detected Object':'Follow_Path'})

        smach.StateMachine.add('Follow_Path', Follow_Path(), 
                               transitions={'Reached Gripping Target':'Grip_Object', 'Reached Releasing Target':'Release_Object'})

        smach.StateMachine.add('Grip_Object', Grip_Object(), 
                               transitions={'Gripped Object':'Follow_Path'})
                               
        smach.StateMachine.add('Release_Object', Release_Object(), 
                               transitions={'Released Object':'Stop'})                             
    # Execute SMACH plan
    outcome = sm.execute()
    #rospy.spin()
    sis.stop()



if __name__ == '__main__':
    main()
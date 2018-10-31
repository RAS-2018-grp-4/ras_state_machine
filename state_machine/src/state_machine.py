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
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
import itertools
import tf
import time
import threading
#from arduino_servo_control.srv import *


FLAG_GRIP = False
FLAG_RELEASE = False
FLAG_START = True
FLAG_DETECT_OBJECT = False
FLAG_PATH_EXECUTION = False
FLAG_DETECT_MISSING_WALL = False

FLAG_RECEIVED = False

FINAL_TARGET_X = 0.0
FINAL_TARGET_Y = 0.0

TARTGET_POSITION = [0.0, 0.0, 0.0]
TARTGET_ORIENTATION = [0.0, 0.0, 0.0]


pub_TARGET_POSE = rospy.Publisher('/target_pose', geometry_msgs.msg.Pose, queue_size=1)
pub_RESET = rospy.Publisher('/odom_reset', std_msgs.msg.Bool, queue_size=1)
pub_STOP = rospy.Publisher('/stop', std_msgs.msg.String, queue_size= 1)

###########################################################
###########################################################
#                    State                                #
###########################################################
###########################################################



#####################################################
#                  Initialization                   #
#####################################################
class Initialization(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Initialization Done'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Initialization')
        #grip = rospy.ServiceProxy('/arduino_servo_control/set_servo_angles', SetServoAngles)
        #grip(0, 180)
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
# class Look_For_Object(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['Detected Object'])

#     def execute(self, userdata):
#         global FLAG_DETECT_OBJECT
#         rospy.loginfo('Executing state Look_For_Object')
#         while not FLAG_DETECT_OBJECT:
#             pass
#         FLAG_DETECT_OBJECT = False
#         return 'Detected Object'


#####################################################
#                 Path_Execution                       #
#####################################################
class Path_Execution(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Detect Object','Reached Gripping Target','Reached Releasing Target', 'Reached Missing Wall'])

    def execute(self, userdata):
        global FLAG_PATH_EXECUTION, FLAG_GRIP, TARTGET_ORIENTATION, TARTGET_POSITION, FLAG_RECEIVED 
        rospy.loginfo('Executing state Path_Execution - Plan_Path')
        while not FLAG_RECEIVED:
            pass
        FLAG_RECEIVED = False

        send_message(TARTGET_ORIENTATION, TARTGET_POSITION)
        rospy.loginfo('Executing state Path_Execution - Path_Following')

        
        while not FLAG_PATH_EXECUTION:
            if FLAG_DETECT_OBJECT and not FLAG_GRIP:
                #send Stop
                return 'Detected Object'
            elif FLAG_DETECT_MISSING_WALL:
                #send stop
                return 'Reached Missing Wall'
            else:
                pass
        FLAG_PATH_EXECUTION = False
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
        global FLAG_GRIP, TARTGET_POSITION, TARTGET_ORIENTATION, FLAG_RECEIVED ,FINAL_TARGET_X, FINAL_TARGET_Y
        rospy.loginfo('Executing state Grip_Object')
        time.sleep(1)
        #grip = rospy.ServiceProxy('/arduino_servo_control/set_servo_angles', SetServoAngles)
        #grip(60, 120)
        time.sleep(1)

        TARTGET_POSITION[0] = FINAL_TARGET_X
        TARTGET_POSITION[1] = FINAL_TARGET_Y

        FLAG_RECEIVED = True

        
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
        #grip = rospy.ServiceProxy('/arduino_servo_control/set_servo_angles', SetServoAngles)
        #grip(0, 180)
        time.sleep(1)
        return 'Released Object'




#####################################################
#                   Add_Wall                        #
#####################################################
class Add_Wall(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Added Wall'])

    def execute(self, userdata):
        global FLAG_GRIP, TARTGET_POSITION, TARTGET_ORIENTATION, FLAG_RECEIVED
        rospy.loginfo('Executing state Add_Wall')
        time.sleep(1)
        # Rotate 180

        FLAG_RECEIVED = True
        time.sleep(1)
        return 'Added Wall'


###########################################################
###########################################################
#                        State                            #
###########################################################
###########################################################


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
#                  start callback                   #
#####################################################
def start_callback(msg):
    global FLAG_START
    if msg.data == True:
        FLAG_START = True
    else :
        pass

#####################################################
#                    flag callback                  #
#####################################################
def flag_callback(msg):
    global FLAG_DETECT_OBJECT,FLAG_PATH_EXECUTION, FLAG_GRIP, FLAG_RELEASE, FLAG_START
    flag = msg.data
    if flag == "detect_object_done" : 
        FLAG_DETECT_OBJECT = True
        #print("detect_object_done")
    elif flag == "path_following_done" :
        FLAG_PATH_EXECUTION = True
        #print("path_following_done")
    elif flag == "grip_done":
        FLAG_GRIP = True
    elif flag == "release_done":
        FLAG_RELEASE = True
    else:
        pass
#####################################################
#              object_position_callback             #
#####################################################
def obj_position_callback(msg):
    global TARTGET_POSITION, TARTGET_ORIENTATION, FLAG_RECEIVED

    FLAG_RECEIVED = True
        #print(msg.data) 
        


#####################################################
#            wall detection callback                #
#####################################################
def wall_detection_callback(msg):
    global FLAG_DETECT_MISSING_WALL
    if msg.data:
        FLAG_DETECT_MISSING_WALL = True
    else:
        pass


#####################################################
#                  goal callback                    #
#####################################################
def goal_callback(msg):
    global TARTGET_POSITION, TARTGET_ORIENTATION, FLAG_RECEIVED

    FINAL_TARGET_X = msg.pose.position.x
    FINAL_TARGET_Y = msg.pose.position.y

    TARTGET_POSITION[0] = msg.pose.position.x
    TARTGET_POSITION[1] = msg.pose.position.y

    FLAG_RECEIVED = True
    #print(msg.data) 
    pass



def main():
    rospy.init_node('state_machine_node')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['Stop'])
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    #rospy.wait_for_service('/arduino_servo_control/set_servo_angles')
    rospy.Subscriber("/Start", std_msgs.msg.Bool, start_callback)
    rospy.Subscriber("/flag_done", String, flag_callback)
    rospy.Subscriber("/object_position", Float32MultiArray, obj_position_callback)
    rospy.Subscriber("/wall_detection", std_msgs.msg.Bool, wall_detection_callback)
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, goal_callback)
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Initialization', Initialization(), 
                               transitions={'Initialization Done':'Standby'})

        smach.StateMachine.add('Standby', Standby(), 
                               transitions={'Receive Start Message':'Path_Execution'})

        # smach.StateMachine.add('Look_For_Object', Look_For_Object(), 
        #                        transitions={'Detected Object':'Path_Execution'})

        smach.StateMachine.add('Path_Execution', Path_Execution(), 
                               transitions={
                               'Detect Object':'Path_Execution',
                               'Reached Gripping Target':'Grip_Object', 
                               'Reached Releasing Target':'Release_Object',
                               'Reached Missing Wall':'Add_Wall'})

        smach.StateMachine.add('Grip_Object', Grip_Object(), 
                               transitions={'Gripped Object':'Path_Execution'})
                               
        smach.StateMachine.add('Release_Object', Release_Object(), 
                               transitions={'Released Object':'Stop'})    
        smach.StateMachine.add('Add_Wall', Add_Wall(), 
                               transitions={'Added Wall':'Path_Execution'})                           
    # Execute SMACH plan
    outcome = sm.execute()
    #rospy.spin()
    sis.stop()



if __name__ == '__main__':
    main()
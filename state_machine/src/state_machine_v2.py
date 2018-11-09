#!/usr/bin/env python
import os
import rospy
import smach
import smach_ros
import rospy
import tf
import math
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from std_msgs.msg import Bool


# ros publisher
PUB_TARGET_POSE = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
PUB_RESET = rospy.Publisher('/odom_reset', Bool, queue_size=1)
PUB_STOP = rospy.Publisher('/path_follower_flag', String, queue_size= 1)
PUB_GRIPPER = rospy.Publisher('/gripper_state', String, queue_size= 1)
pub_GRIPPED = rospy.Publisher('/gripped_done', Bool, queue_size=1)


########################################
#             Robot State              #
########################################
class robot:
    def __init__(self):
        self.final_position = [0.0, 0.0, 0.0]
        self.moving_position = [0.0, 0.0, 0.0]   # 
        self.gripping = False                    # 
        self.going_to_object = False 





###########################################################
###########################################################
#                    State                                #
###########################################################
###########################################################

########################################
#             Initialization           #
########################################
class Initialization(smach.State):

    def __init__(self):

        smach.State.__init__(self, outcomes=['Initialization Done'],
                                   input_keys=['robot_state'],
                                   output_keys=['robot_state'])

    def execute(self, userdata):

        rospy.loginfo('Executing state Initialization')

        send_gripper_message("close")

        send_reset_message()

        return 'Initialization Done'


########################################
#                Standby               #
########################################
class Standby(smach.State):

    def __init__(self):

        smach.State.__init__(self, outcomes=['Receive Start Message'],
                                   input_keys=['robot_state'],
                                   output_keys=['robot_state'])

        # ROS subscriber
        rospy.Subscriber("/start", Bool, self.start_callback)

        # flag
        self.flag_start = True                  # True when receive start signal

        #
        self.target_position = [0.9, 0.9, 0.0]
        
    ##################################
    #        Callback Function       #
    ##################################
    def start_callback(self, msg):
        if msg.data:
            self.flag_start = True


    #  
    def execute(self, userdata):

        rospy.loginfo('Executing state Standby')

        #
        while not self.flag_start:
            pass
        self.flag_start = False
        #
        userdata.robot_state.final_position = self.target_position
        userdata.robot_state.moving_position = self.target_position
        return 'Receive Start Message'

########################################
#              Path_Execution          #
########################################
class Path_Execution(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Detected Object',
                                             'Reached Target',
                                             'Reached Releasing Target', 
                                             'Reached Missing Wall'],
                                   input_keys=['robot_state'],
                                   output_keys=['robot_state'])

        # Ros subscriber
        rospy.Subscriber("/object_position_map", PointStamped, self.obj_position_callback)
        rospy.Subscriber("/flag_done", String, self.flag_callback)
        rospy.Subscriber("/wall_detection", Bool, self.wall_detection_callback)

        # flag
        self.flag_detect_object = False                  # True when detect object
        self.flag_path_execution = False                 # True when path following done
        self.flag_detect_missing_wall = False            # True when detect missing wall
        self.flag_object_position_received = False       # True when receive object position

        #
        self.object_position = [0.0, 0.0, 0.0]


    ##################################
    #        Callback Function       #
    ##################################

    # flag callback
    def flag_callback(self, msg):

        flag = msg.data
        if flag == "detect_object_done" : 
            self.flag_detect_object = True
            # if not State_Machine.FLAG_GRIPPED :
            #     msg_string = String()
            #     msg_string.data = "open"
            #     State_Machine.pub_GRIPPER.publish(msg_string)
        elif flag == "path_following_done" :
            self.flag_path_execution = True
        else:
            pass

    # object_position_callback 
    def obj_position_callback(self, msg):
        if not math.isnan(msg.point.x):
            self.object_position[0] = msg.point.x
            self.object_position[1] = msg.point.y
            self.flag_object_position_received = True
        
    # wall detection 
    def wall_detection_callback(self, msg):
        if msg.data:
        #    State_Machine.FLAG_DETECT_MISSING_WALL = True
            pass
        else:
            pass


    def execute(self, userdata):
        rospy.loginfo('Executing state Path_Execution - Plan_Path')
        userdata.robot_state.moving_position = userdata.robot_state.final_position
        rospy.loginfo("Going to position %.2lf %.2lf", userdata.robot_state.moving_position[0], 
                                                       userdata.robot_state.moving_position[1])


        send_position_message(userdata.robot_state.moving_position)

        gripping = userdata.robot_state.gripping

        rospy.loginfo('Executing state Path_Execution - Path_Following')

        while not self.flag_path_execution:
            if self.flag_detect_object and (not gripping) :
                #send Stop
                send_stop_message()
                send_gripper_message("open")
                return 'Detected Object'
            elif self.flag_detect_missing_wall:
                #send stop
                send_stop_message()
                return 'Reached Missing Wall'
            else:
                pass
            #FLAG_GO_TO_OBJECT = False

        self.flag_path_execution = False

        if gripping:
            return 'Reached Target'
        else:
            return 'Reached Releasing Target'
            


########################################
#      Going To Position Of Object     #
########################################
class Going_To_Position_Of_Object(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Reached Gripping Target', 
                                             'Reached Missing Wall'],
                                   input_keys=['robot_state'],
                                   output_keys=['robot_state'])

        # Ros subscriber
        rospy.Subscriber("/flag_done", String, self.flag_callback)
        rospy.Subscriber("/wall_detection", Bool, self.wall_detection_callback)

        # flag
        self.flag_path_execution = False                 # True when path following done
        self.flag_detect_missing_wall = False            # True when detect missing wall
        #
        self.object_position = [0.0, 0.0, 0.0]



    ##################################
    #        Callback Function       #
    ##################################

    # flag callback
    def flag_callback(self, msg):

        flag = msg.data
        if flag == "detect_object_done" : 
            pass
        elif flag == "path_following_done" :
            self.flag_path_execution = True
        else:
            pass
        
    # wall detection 
    def wall_detection_callback(self, msg):
        if msg.data:
        #    State_Machine.FLAG_DETECT_MISSING_WALL = True
            pass
        else:
            pass


    def execute(self, userdata):
        rospy.loginfo('Executing state Going To Position Of Object - Plan_Path')
        rospy.loginfo("Going to position %.2lf %.2lf", userdata.robot_state.moving_position[0], 
                                                       userdata.robot_state.moving_position[1])
        userdata.robot_state.going_to_object = True
        send_position_message(userdata.robot_state.moving_position)


        rospy.loginfo('Executing state Going To Position Of Object - Path_Following')

        while not self.flag_path_execution:
            if self.flag_detect_missing_wall:
                #send stop
                send_stop_message()
                return 'Reached Missing Wall'
            else:
                pass

        self.flag_path_execution = False

        return 'Reached Gripping Target'

########################################
#              Grip_Object             #
########################################
class Grip_Object(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Gripped Object'],
                                   input_keys=['robot_state'],
                                   output_keys=['robot_state'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Grip_Object')

        send_gripper_message("grip")
        # msg = Bool()
        # msg.data = True
        # State_Machine.pub_GRIPPED.publish(msg)

        userdata.robot_state.gripping = True
        userdata.robot_state.going_to_object = False

        return 'Gripped Object'

########################################
#        Release_Object                #
########################################
class Release_Object(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Released Object'],
                                   input_keys=['robot_state'],
                                   output_keys=['robot_state'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Release_Object')

        send_gripper_message("open")
        userdata.robot_state.gripping = False
        # msg = Bool()
        # msg.data = False
        # State_Machine.pub_GRIPPED.publish(msg)

        return 'Released Object'




########################################
#         Add_Wall                     #
########################################
class Add_Wall(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Added Wall (Druing Path Execution)','Added Wall (Druing Going To Object)'],
                                   input_keys=['robot_state'],
                                   output_keys=['robot_state'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Add_Wall')
        # Rotate 180
        rospy.sleep(1)
        if not userdata.robot_state.going_to_object:
            return 'Added Wall (Druing Path Execution)'
        else:
            return 'Added Wall (Druing Going To Object)'


###########################################################
###########################################################
#                        State                            #
###########################################################
###########################################################



##################################
#       Publisher Function       #
##################################

# Send_Message 
def send_position_message(target_position):
    global PUB_TARGET_POSE
    rospy.sleep(2)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = target_position[0]
    pose.pose.position.y = target_position[1]
    pose.pose.position.z = 0
    (r, p, y, w) = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
    pose.pose.orientation.z = y
    pose.pose.orientation.w = w
    PUB_TARGET_POSE.publish(pose)
    rospy.sleep(2)

# send gripper action
def send_gripper_message(action):
    global PUB_GRIPPER
    rospy.sleep(2)
    msg_string = String()
    msg_string.data = action
    PUB_GRIPPER.publish(msg_string)
    rospy.sleep(2)

# send odom reset message
def send_reset_message():
    rospy.sleep(2)
    global PUB_RESET
    msg_bool = Bool()
    msg_bool.data = True
    PUB_RESET.publish(msg_bool)
    rospy.sleep(2)

# send stop message
def send_stop_message():
    rospy.sleep(2)
    global PUB_RESET
    msg_string = String()
    msg_string.data = 'STOP'
    PUB_STOP.publish(msg_string)
    rospy.sleep(2)

def main():
    rospy.init_node('state_machine_node')

    sm = smach.StateMachine(outcomes=['Stop'])
    sm.userdata.robot_state = robot()
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Initialization', Initialization(), 
                               transitions={'Initialization Done':'Standby'},
                               remapping={'robot_state':'robot_state', 
                                          'robot_state':'robot_state'})

        smach.StateMachine.add('Standby', Standby(), 
                               transitions={'Receive Start Message':'Path_Execution'},
                                remapping={'robot_state':'robot_state', 
                                           'robot_state':'robot_state'})

        smach.StateMachine.add('Path_Execution', Path_Execution(), 
                               transitions={
                               'Detected Object':'Going_To_Position_Of_Object',
                               'Reached Target':'Stop',
                               'Reached Releasing Target':'Release_Object',
                               'Reached Missing Wall':'Add_Wall'},
                               remapping={'robot_state':'robot_state', 
                                          'robot_state':'robot_state'})

        smach.StateMachine.add('Going_To_Position_Of_Object', Going_To_Position_Of_Object(), 
                               transitions={
                               'Reached Gripping Target':'Grip_Object',
                               'Reached Missing Wall':'Add_Wall'},
                               remapping={'robot_state':'robot_state', 
                                          'robot_state':'robot_state'})

        smach.StateMachine.add('Grip_Object', Grip_Object(), 
                               transitions={'Gripped Object':'Path_Execution'},
                               remapping={'robot_state':'robot_state', 
                                          'robot_state':'robot_state'})
                               
        smach.StateMachine.add('Release_Object', Release_Object(), 
                               transitions={'Released Object':'Stop'},
                               remapping={'robot_state':'robot_state', 
                                          'robot_state':'robot_state'})   

        smach.StateMachine.add('Add_Wall', Add_Wall(), 
                               transitions={'Added Wall (Druing Path Execution)':'Path_Execution',
                                            'Added Wall (Druing Going To Object)':'Going_To_Position_Of_Object'},
                               remapping={'robot_state':'robot_state', 
                                          'robot_state':'robot_state'})                          
    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

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
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Bool


##########################################
#              Robot State               #
##########################################
class robot:
    def __init__(self):
        self.final_position = [0.2, 0.2, 0.0]
        self.moving_position = [0.0, 0.0, 0.0]   
        self.starting_position = [0.2, 0.2, 0.0] 
        self.gripping = False                   
        self.going_to_object = False 
        self.clos_to_object = False
        self.going_to_starting_position = False             

##########################################
#        State : Initialization          #
##########################################
class Initialization(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Initialization Done'],
                                   input_keys=['robot_state'],
                                   output_keys=['robot_state'])
        
        # publisher
        self.pub_reset = rospy.Publisher('/odom_reset', Bool, queue_size=1)
        self.pub_gripper = rospy.Publisher('/gripper_state', String, queue_size= 1)

    ###############################
    #      Publisher Function     #
    ###############################       
    def send_reset_message(self):
        rospy.sleep(2)
        msg_bool = Bool()
        msg_bool.data = True
        self.pub_reset.publish(msg_bool)
        rospy.sleep(2)

    def send_gripper_message(self,action):
        rospy.sleep(2)
        msg_string = String()
        msg_string.data = action
        self.pub_gripper.publish(msg_string)
        rospy.sleep(2)

    def execute(self, userdata):
        rospy.loginfo('Executing state Initialization')

        self.send_gripper_message("close")

        self.send_reset_message()

        return 'Initialization Done'

##########################################
#        State : Standby                 #
##########################################
class Standby(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Receive Start Message'],
                                   input_keys=['robot_state'],
                                   output_keys=['robot_state'])

        # subscriber
        rospy.Subscriber("/start", Bool, self.start_callback)

        #flag
        self.flag_start = True                  # True when receive start signal

        #self.target_position = [0.9, 0.9, 0.0]
        self.target_position = [0.25, 1.2, 0.0]
        #self.target_position = [1.6, 0.73, 0.0]
    def start_callback(self, msg):
        if msg.data:
            self.flag_start = True
 
    def execute(self, userdata):
        rospy.loginfo('Executing state Standby')

        # wait until receive start signal
        while not self.flag_start:
            pass
        self.flag_start = False

        #speaker

        #set final destination and current moving target
        userdata.robot_state.final_position = self.target_position
        userdata.robot_state.moving_position = self.target_position
        userdata.robot_state.going_to_object = True
        return 'Receive Start Message'

##########################################
#         State : Path_Execution         #
##########################################
class Path_Execution(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Reached Gripping Target',
                                             'Reached Staring Position', 
                                             'Reached Missing Wall',
                                             'Reached Missing Rubble'],
                                   input_keys=['robot_state'],
                                   output_keys=['robot_state'])

        # subscriber
        rospy.Subscriber("/best_object", PointStamped, self.obj_position_callback)
        rospy.Subscriber("/flag_done", String, self.flag_callback)
        rospy.Subscriber("/flag_path_follower", String, self.flag_path_callback)
        rospy.Subscriber("/wall_detection", Bool, self.wall_detection_callback)
        #rospy.Subscriber("/wall_detection", Bool, self.wall_detection_callback)
        self.pub_gripper = rospy.Publisher('/gripper_state', String, queue_size= 1)
        # publisher
        self.pub_pose = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.pub_gripper = rospy.Publisher('/gripper_state', String, queue_size= 1)
        self.pub_stop = rospy.Publisher('/path_follower_flag', String, queue_size= 1)


        # flag
        self.flag_detect_object = False                  # True when detect object
        self.flag_path_execution = False                 # True when path following done
        self.flag_detect_missing_wall = False            # True when detect missing wall
        self.flag_object_position_received = False       # True when receive object position
        self.flag_detect_missing_rubble = False          # True when detect missing rubble
        self.flag_close_to_object = False
        # position of object
        self.object_position = [0.0, 0.0, 0.0]
        

        


    ###############################
    #         Callbacks           #
    ###############################
    def flag_callback(self, msg):
        flag = msg.data

        # when detect object
        if flag == "detect_object_done" : 
            self.flag_detect_object = True

        # when reach target position
        elif flag == "path_following_done" :
            self.flag_path_execution = True
        else:
            pass


    def flag_path_callback(self, msg):
        flag = msg.data
        # when detect object
        if flag == "CLOSE_TO_TARGET" : 
            self.flag_close_to_object = True


    def obj_position_callback(self, msg):
        if not math.isnan(msg.point.x):
            self.object_position[0] = msg.point.x
            self.object_position[1] = msg.point.y
            self.flag_object_position_received = True
        
    def wall_detection_callback(self, msg):
        if msg.data:
            self.flag_detect_missing_wall = True
        else:
            pass

    def rubber_detection_callback(self, msg):
        if msg.data:
            self.flag_detect_missing_wall = True
        else:
            pass


    ###############################
    #      Publisher Function     #
    ###############################
    def send_gripper_message(self,action):
        rospy.sleep(2)
        msg_string = String()
        msg_string.data = action
        self.pub_gripper.publish(msg_string)
        rospy.sleep(2)

    def send_position_message(self, target_position):
        rospy.sleep(2)
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = target_position[0]
        pose.pose.position.y = target_position[1]
        pose.pose.position.z = 0
        (r, p, y, w) = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        pose.pose.orientation.z = y
        pose.pose.orientation.w = w
        self.pub_pose.publish(pose)
        rospy.sleep(2)

    def send_stop_message(self):
        #rospy.sleep(2)
        msg_string = String()
        msg_string.data = "STOP"
        self.pub_stop.publish(msg_string)
        rospy.sleep(5)

    def send_gripper_message(self,action):
        rospy.sleep(2)
        msg_string = String()
        msg_string.data = action
        self.pub_gripper.publish(msg_string)
        rospy.sleep(2)

    ###############################
    #         Execution           #
    ###############################
    def execute(self, userdata):
        rospy.loginfo('Executing state Path_Execution - Plan_Path')

        # get robot current state
        gripping = userdata.robot_state.gripping
        going_to_object = userdata.robot_state.going_to_object
        going_to_starting_position = userdata.robot_state.going_to_starting_position

        # send current target position to path planner
        target = ""
        if going_to_object:
            target = "Object"
            while not self.flag_object_position_received:
                pass
            self.flag_object_position_received = False
            userdata.robot_state.moving_position[0] = self.object_position[0]
            userdata.robot_state.moving_position[1] = self.object_position[1]
        elif going_to_starting_position:
            target = "Staring Position"
            userdata.robot_state.moving_position[0] = userdata.robot_state.starting_position[0]
            userdata.robot_state.moving_position[1] = userdata.robot_state.starting_position[1]
        rospy.loginfo("Going to %s : %.2lf %.2lf", target, 
                                                   userdata.robot_state.moving_position[0], 
                                                   userdata.robot_state.moving_position[1])
        self.send_position_message(userdata.robot_state.moving_position)

        rospy.loginfo('Executing state Path_Execution - Path_Following')

        # when not reach target position
        while not self.flag_path_execution:

            if self.flag_detect_missing_wall:

                # send stop
                self.send_stop_message()
                self.flag_detect_missing_wall = False
                return 'Reached Missing Wall'
                
            elif self.flag_detect_missing_rubble:

                # send stop
                self.send_stop_message()
                return 'Reached Missing Rubble'
            else:
                pass

            if self.flag_close_to_object:
                self.flag_close_to_object = False
                print("open")
                self.send_gripper_message('open')
            #FLAG_GO_TO_OBJECT = False

        self.flag_path_execution = False

        if going_to_object:
            return 'Reached Gripping Target'
        elif going_to_starting_position:
            return 'Reached Staring Position'
        else:
            pass           

##########################################  
#          State : Grip_Object           #
##########################################
class Grip_Object(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Gripped Object'],
                                   input_keys=['robot_state'],
                                   output_keys=['robot_state'])

        # publisher
        self.pub_gripper = rospy.Publisher('/gripper_state', String, queue_size= 1)
        self.pub_gripped = rospy.Publisher('/flag_gripped', String, queue_size= 1)

    ###############################
    #      Publisher Function     #
    ###############################
    def send_gripper_message(self,action):
        #rospy.sleep(2)
        msg_string = String()
        msg_string.data = action
        self.pub_gripper.publish(msg_string)
        rospy.sleep(2)

    def send_gripped_message(self):
        #rospy.sleep(2)
        msg_string = String()
        msg_string.data = "gripped"
        self.pub_gripped.publish(msg_string)
        rospy.sleep(2)

    def execute(self, userdata):
        rospy.loginfo('Executing state Grip_Object')

        self.send_gripper_message("grip")

        userdata.robot_state.gripping = True
        userdata.robot_state.going_to_object = False
        userdata.robot_state.moving_position = userdata.robot_state.final_position
        userdata.robot_state.going_to_starting_position = True
        self.send_gripped_message()
        return 'Gripped Object'

##########################################  
#         State : Release_Object         #
##########################################
class Release_Object(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Released Object'],
                                   input_keys=['robot_state'],
                                   output_keys=['robot_state'])

        # publisher
        self.pub_gripper = rospy.Publisher('/gripper_state', String, queue_size= 1)
        self.pub_obj_done = rospy.Publisher('/object_done', String, queue_size= 1)
        self.pub_vel = rospy.Publisher('/keyboard/vel', Twist, queue_size=1)

    ###############################
    #      Publisher Function     #
    ###############################
    def send_gripper_message(self,action):
        rospy.sleep(2)
        msg_string = String()
        msg_string.data = action
        self.pub_gripper.publish(msg_string)
        rospy.sleep(2)

    def send_object_message(self):
        #rospy.sleep(2)
        msg_string = String()
        msg_string.data = "Finish"
        self.pub_obj_done.publish(msg_string)
        rospy.sleep(5)


    def back(self):
            vel = Twist()
            vel.linear.x = -0.02
            vel.linear.y = 0.0
            vel.linear.z = 0.0
            vel.angular.x = 0.0
            vel.angular.y = 0.0
        
            # rotate
            vel.angular.z = 0.0
            self.pub_vel.publish(vel)
            rospy.loginfo('Rotating')
            rospy.sleep(4)
            rospy.loginfo('Rotate Finished')
            # stop
            vel.angular.z = 0.0
            self.pub_vel.publish(vel)

    def execute(self, userdata):
        rospy.loginfo('Executing state Release_Object')

        self.send_gripper_message("open")
        self.send_object_message
        userdata.robot_state.gripping = False
        userdata.robot_state.going_to_object = True
        userdata.robot_state.going_to_starting_position = False
        self.back()
        return 'Released Object'

##########################################  
#         State : Mapping_Wall           #
##########################################
class Mapping_Wall(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Mapped Wall'],
                                   input_keys=['robot_state'],
                                   output_keys=['robot_state'])
        
        self.pub_vel = rospy.Publisher('/keyboard/vel', Twist, queue_size=1)

    def rotate_180(self):
        vel = Twist()
        vel.linear.x = 0.0
        vel.linear.y = 0.0
        vel.linear.z = 0.0
        vel.angular.x = 0.0
        vel.angular.y = 0.0
        
        # rotate 4 sec
        vel.angular.z = 0.3
        self.pub_vel.publish(vel)
        rospy.sleep(5)

        # stop
        vel.angular.z = 0.0
        self.pub_vel.publish(vel)

    def execute(self, userdata):
        rospy.loginfo('Executing state Mapping_Wall')
        rospy.sleep(1)

        # Rotate 180
        self.rotate_180()

        return 'Mapped Wall'


##########################################  
#        State : Mapping_Rubble          #
##########################################
class Mapping_Rubble(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Mapped Rubble'],
                                   input_keys=['robot_state'],
                                   output_keys=['robot_state'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Mapping_Rubble')
        # Rotate 180
        rospy.sleep(1)
        return 'Mapped Rubble'

##################################
#          Main Function         #
##################################
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
                               transitions={'Reached Staring Position':'Release_Object',
                                            'Reached Gripping Target':'Grip_Object',
                                            'Reached Missing Wall':'Mapping_Wall',
                                            'Reached Missing Rubble':'Mapping_Rubble'},
                               remapping={'robot_state':'robot_state', 
                                          'robot_state':'robot_state'})
    
        smach.StateMachine.add('Grip_Object', Grip_Object(), 
                               transitions={'Gripped Object':'Path_Execution'},
                               remapping={'robot_state':'robot_state', 
                                          'robot_state':'robot_state'})
                               
        smach.StateMachine.add('Release_Object', Release_Object(), 
                               transitions={'Released Object':'Path_Execution'},
                               remapping={'robot_state':'robot_state', 
                                          'robot_state':'robot_state'})   

        smach.StateMachine.add('Mapping_Wall', Mapping_Wall(), 
                               transitions={'Mapped Wall':'Path_Execution'},
                               remapping={'robot_state':'robot_state', 
                                          'robot_state':'robot_state'})        

        smach.StateMachine.add('Mapping_Rubble', Mapping_Rubble(), 
                               transitions={'Mapped Rubble':'Path_Execution'},
                               remapping={'robot_state':'robot_state', 
                                          'robot_state':'robot_state'})   

                                          

    # Execute SMACH plan
    outcome = sm.execute()

    # pub_GRIPPED = rospy.Publisher('/gripped_done', Bool, queue_size=1)
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

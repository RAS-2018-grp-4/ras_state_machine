#!/usr/bin/env python
import os
import rospy
import smach
import smach_ros
import rospy
import tf
import math
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan

##########################################
#              Robot State               #
##########################################
class robot:
    def __init__(self):
        self.final_position = [0.0, 0.0, 0.0]
        self.moving_position = [0.0, 0.0, 0.0]   
        self.starting_position = [0.2, 0.2, 0.0] 
        self.gripping = False                   
        self.going_to_object = False 
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
        self.pub_wall_disable = rospy.Publisher('/wall_disable', String, queue_size=1)


    ###############################
    #      Publisher Function     #
    ###############################
    def send_disable_message(self,msg_str):
        rospy.sleep(1)
        msg_string = String()
        msg_string.data = msg_str
        self.pub_wall_disable.publish(msg_string)
      
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
        self.send_disable_message("able")
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

        return 'Receive Start Message'

##########################################
#           State : Explore              #
##########################################
class Explore(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Finished Path',
                                             'Reached Missing Object',
                                             'Reached Missing Wall',
                                             'Reached Missing Battery',
                                             'Abort_Path_Following',
                                             'Stop Explore',
                                             'Start Rotate'],
                                   input_keys=['robot_state'],
                                   output_keys=['robot_state'])

        # subscriber
        rospy.Subscriber("/best_object", PointStamped, self.obj_position_callback)
        rospy.Subscriber("/flag_done", String, self.flag_callback)
        rospy.Subscriber("/wall_detection", Bool, self.wall_detection_callback)
        rospy.Subscriber("/next_traget",PoseStamped,self.next_traget_callback)
        rospy.Subscriber("/flag_pathplanner", String, self.path_planner_callback)
        rospy.Subscriber("/battery_detected", String, self.battery_callback)
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)

        # publisher
        self.pub_pose = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.pub_gripper = rospy.Publisher('/gripper_state', String, queue_size= 1)
        self.pub_stop = rospy.Publisher('/path_follower_flag', String, queue_size= 1)
        self.pub_vel = rospy.Publisher('/keyboard/vel', Twist, queue_size=1)


        # flag
        self.flag_detect_object = False                  # True when detect object
        self.flag_path_execution = False                 # True when path following done
        self.flag_detect_missing_wall = False            # True when detect missing wall
        self.flag_object_position_received = False       # True when receive object position
        self.flag_next_traget_received = False
        self.flag_path_planner = False
        self.flag_battery_detected = False
        self.flag_empty = False


        # position of object
        self.object_position = [0.0, 0.0, 0.0]
        
        self.next_pose = [0.0,0.0,0.0]


    ###############################
    #         Callbacks           #
    ###############################
    def battery_callback(self,msg):
        if msg.data == "BATTERY_DETECTED":
            self.flag_battery_detected = True
        else:
            pass


    def path_planner_callback(self,msg):
        if msg.data == "NO_PATH_FOUND":
            self.flag_path_planner = True
        else:
            pass

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


    def next_traget_callback(self, msg):
        self.next_pose[0] = msg.pose.position.x
        self.next_pose[1] = msg.pose.position.y
        self.flag_next_traget_received = True

    def laser_callback(self,scan):
        count = (int)(scan.scan_time / scan.time_increment)
        num = 0
        sum = 0.0
        flag = True
        for i in range(0, count):
            if scan.ranges[i] !=float("inf"):
                sum = sum + scan.ranges[i]
                num = num + 1
                if scan.ranges[i]< 0.25:
                    flag = False
                    break

        if sum/num > 0.4 and flag:
            self.flag_empty = True
        else:
            self.flag_empty = False  
    ###############################
    #      Publisher Function     #
    ###############################
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

    def rotate(self):
            vel = Twist()
            vel.linear.x = 0.0
            vel.linear.y = 0.0
            vel.linear.z = 0.0
            vel.angular.x = 0.0
            vel.angular.y = 0.0
        
            # rotate
            vel.angular.z = 0.3
            self.pub_vel.publish(vel)
            rospy.loginfo('Rotating')
            rospy.sleep(10)
            rospy.loginfo('Rotate Finished')
            # stop
            vel.angular.z = 0.0
            self.pub_vel.publish(vel)


    ###############################
    #         Execution           #
    ###############################
    def execute(self, userdata):
        rospy.loginfo('Executing state Explore - Plan_Path')

        # get robot current state
        gripping = userdata.robot_state.gripping
        going_to_object = userdata.robot_state.going_to_object
        going_to_starting_position = userdata.robot_state.going_to_starting_position

        # send current target position to path planner
        rospy.loginfo("Going to : %.2lf %.2lf", userdata.robot_state.moving_position[0], 
                                                   userdata.robot_state.moving_position[1])
        self.send_position_message(userdata.robot_state.moving_position)

        rospy.loginfo('Executing state Explore - Path_Following')

        # when not reach target position
        while not self.flag_path_execution:


            if self.flag_detect_missing_wall:

                # send stop
                self.send_stop_message()
                self.flag_detect_missing_wall = False

                # abort plan following and open gripper 
                rospy.loginfo('Abort Path_Following')

                return 'Reached Missing Wall'
            else:
                pass
            
                
            if self.flag_battery_detected:
                self.flag_battery_detected = False
                # send stop
                self.send_stop_message()
                rospy.loginfo('Abort Path_Following')
                return 'Reached Missing Battery'
            else:
                pass

            if self.flag_path_planner:
                break
            else:
                pass
            #FLAG_GO_TO_OBJECT = False


            #if self.flag_empty:
            #    self.flag_empty = False
            #    self.send_stop_message()
            #    return 'Start Rotate'

        self.flag_path_execution = False
        if self.flag_path_planner:
            rospy.loginfo('Abort Path Planner')
            self.flag_path_planner = False
        else:
            rospy.loginfo('Finished Path_Following')

        while not self.flag_next_traget_received:
            pass
        self.flag_next_traget_received = False
        userdata.robot_state.moving_position[0] = self.next_pose[0]
        userdata.robot_state.moving_position[1] = self.next_pose[1]
        rospy.loginfo('Received Next Target')
        if self.flag_path_planner:
            return 'Abort_Path_Following'
        else:
            return 'Finished Path'           



##########################################  
#          State : Rotate               #
##########################################
class Rotate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Rotate Finish'],
                                   input_keys=['robot_state'],
                                   output_keys=['robot_state'])

        # publisher
        self.pub_vel = rospy.Publisher('/keyboard/vel', Twist, queue_size=1)
        self.pub_wall_disable = rospy.Publisher('/wall_disable', String, queue_size=1)
    ###############################
    #      Publisher Function     #
    ###############################


    def send_disable_message(self,msg_str):
        rospy.sleep(1)
        msg_string = String()
        msg_string.data = msg_str
        self.pub_wall_disable.publish(msg_string)
        rospy.sleep(2)
    def rotate(self):
            vel = Twist()
            vel.linear.x = 0.0
            vel.linear.y = 0.0
            vel.linear.z = 0.0
            vel.angular.x = 0.0
            vel.angular.y = 0.0
        
            # rotate
            vel.angular.z = 0.3
            self.pub_vel.publish(vel)
            rospy.loginfo('Rotating')
            rospy.sleep(10)
            rospy.loginfo('Rotate Finished')
            # stop
            vel.angular.z = 0.0
            self.pub_vel.publish(vel)

    def execute(self, userdata):
        rospy.loginfo('Executing state Rotate')
        self.send_disable_message("disable")
        self.rotate()
        self.send_disable_message("able")
        return 'Rotate Finish'


##########################################  
#         State : Mapping_Wall           #
##########################################
class Mapping_Wall(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Mapped Wall'],
                                   input_keys=['robot_state'],
                                   output_keys=['robot_state'])
        

    def execute(self, userdata):
        rospy.loginfo('Executing state Mapping_Wall')
        rospy.sleep(8)

        # Rotate 180
        #self.rotate_180()

        return 'Mapped Wall'


##########################################  
#        State : Mapping_Battery          #
##########################################
class Mapping_Battery(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Mapped Battery'],
                                   input_keys=['robot_state'],
                                   output_keys=['robot_state'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Mapping_Battery')
        # Rotate 180
        rospy.sleep(1)
        return 'Mapped Battery'



##########################################  
#        State : Mapping_Object          #
##########################################
class Mapping_Object(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Mapped Object'],
                                   input_keys=['robot_state'],
                                   output_keys=['robot_state'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Mapping_Object')
        # Rotate 180
        rospy.sleep(1)
        return 'Mapped Object'

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
                               transitions={'Receive Start Message':'Explore'},
                                remapping={'robot_state':'robot_state', 
                                           'robot_state':'robot_state'})

        smach.StateMachine.add('Explore', Explore(), 
                               transitions={'Finished Path':'Rotate',
                                            'Reached Missing Object':'Mapping_Object',
                                            'Reached Missing Wall':'Mapping_Wall',
                                            'Reached Missing Battery':'Mapping_Battery',
                                            'Abort_Path_Following':'Explore',
                                            'Start Rotate':'Rotate',
                                            'Stop Explore':'Stop'},
                               remapping={'robot_state':'robot_state', 
                                          'robot_state':'robot_state'})  
        smach.StateMachine.add('Rotate', Rotate(), 
                               transitions={'Rotate Finish':'Explore'},
                               remapping={'robot_state':'robot_state', 
                                          'robot_state':'robot_state'})  

        smach.StateMachine.add('Mapping_Wall', Mapping_Wall(), 
                               transitions={'Mapped Wall':'Explore'},
                               remapping={'robot_state':'robot_state', 
                                          'robot_state':'robot_state'})        

        smach.StateMachine.add('Mapping_Object', Mapping_Object(), 
                               transitions={'Mapped Object':'Explore'},
                               remapping={'robot_state':'robot_state', 
                                          'robot_state':'robot_state'}) 

        smach.StateMachine.add('Mapping_Battery', Mapping_Battery(), 
                               transitions={'Mapped Battery':'Explore'},
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

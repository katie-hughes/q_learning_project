#!/usr/bin/env python3

import rospy
# import the moveit_commander, which allows us to control the arms
import moveit_commander
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

class Robot(object):

    def __init__(self):

        # initialize this node
        rospy.init_node('movearm')

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

                #Declare node as a subscriber to the scan topic and
        # set self.process_scan as the function to be used for callback
        rospy.Subscriber("/scan", LaserScan, self.process_scan)

        #Get a publisher to the cmd_vel topic
        self.twist_pub = rospy.Publisher ("/cmd_vel", Twist, queue_size = 10)

        #Create a default twist msg (all values 0)
        lin = Vector3()
        ang = Vector3()
        self.twist = Twist(linear = lin, angular = ang)
        self.turning = False
        self.moving = False
        
    def open_gripper(self):
        gripper_joint_goal = self.move_group_gripper.get_current_joint_values()
        gripper_joint_goal[0] = 0.019
        gripper_joint_goal[1] = 0.019
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()        
        
    def close_gripper(self):
        gripper_joint_goal = self.move_group_gripper.get_current_joint_values()
        gripper_joint_goal = [0.0, 0.0]
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()        

    def reach_dumbbell(self):
        arm_joint_goal = self.move_group_arm.get_current_joint_values()
        arm_joint_goal[0]=0.0
        arm_joint_goal[1]=0.75
        arm_joint_goal[2]=-0.4
        arm_joint_goal[3]=-0.35
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()

    def lift_dumbbell(self):
        arm_joint_goal = self.move_group_arm.get_current_joint_values()
        arm_joint_goal=[0.0,-.8,-0.2,0.3]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()

    def set_dumbbell(self):
        arm_joint_goal = self.move_group_arm.get_current_joint_values()
        arm_joint_goal=[0.0,0.75,-0.75,0.0]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()

    def face_dumbbell(self,data):
        rotspeed = math.pi/4 # 180 degrees per second
        range_data = np.array(data.ranges) #convert to numpy array
        minval = min(range_data) # closest point in space
        mindir = np.argmin(range_data) # find direction to db
        if (mindir == 0):
            self.turning = False
            self.twist.angular.z = 0
            self.twist_pub.publish(Twist())
        elif mindir < 180:
            self.turning = True
            self.twist.angular.z = rotspeed*(mindir)/180
        else:
            self.turning = True
            self.twist.angular.z = rotspeed*(mindir-360)/180
            
    def approach_dumbbell(self,data):
        distance = 0.25 # stay 1/2 meters away from the dumbbell
        speed = 0.01 # m/s
        range_data = np.array(data.ranges) #convert to numpy array
        minval = min(range_data) # closest point in space
        mindir = np.argmin(range_data) # find direction to db
        if data.ranges[0] > distance: # if we are far enough from the db
            self.twist.linear.x = speed
            self.moving = True
            self.turning = True
        else:
            self.twist.linear.x = 0
            self.twist_pub.publish(Twist())
            self.moving = False
            self.turning = False


            
    def process_scan(self,data):
        if (self.turning == True):
            self.face_dumbbell(data)
        elif (self.moving == True):
            self.approach_dumbbell(data)
        self.twist_pub.publish(self.twist)
        
    def run(self):
        rate = rospy.Rate(1)
        rate.sleep()
        self.moving = False # boolean for moving toward db
        self.turning = False # boolean for turning toward db
        self.twist_pub.publish(Twist()) # stop moving
        print("resetting position...")
        self.lift_dumbbell()
        print("open gripper...")
        
        print("setting orientation:")
        self.turning = True
        while (self.turning == True):
            rate.sleep() # wait for orienting toward db

        print("resetting position...")
        self.lift_dumbbell()
        print("open gripper...")
        self.open_gripper()
        print("reach_dumbbell..")
        self.reach_dumbbell()

        print("moving toward db...")
        self.moving = True
        while (self.moving == True):
            rate.sleep() # Wait for movement to stop
        
        print("close gripper")
        self.close_gripper()
        rate.sleep()
        
        print("lift dumbbell")
        self.lift_dumbbell()
        rate.sleep()

        self.twist.linear.x = .25  # move foward
        self.twist_pub.publish(self.twist)
        rate.sleep()
        self.twist.linear.x = 0
        self.twist_pub.publish(Twist())  # stop
        
        print("putting it back down..")
        self.reach_dumbbell()
        self.open_gripper()
        
        rospy.spin()

if __name__=="__main__":

    node=Robot()
    node.run()

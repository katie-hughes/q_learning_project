#!/usr/bin/env python3

import rospy

from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from q_learning_project.msg import RobotMoveDBToBlock, QMatrix, QLearningReward

import time

from tf.transformations import quaternion_from_euler, euler_from_quaternion


class Q_Learning(object):

    def __init__(self):

        # initialize this node
        rospy.init_node('turtlebot3_q_learning')

        # Create a publisher to the qmatrix 
        self.q_matrix_pub = rospy.Publisher("/q_learning/QMatix", QMatrix, queue_size=1)
        # self.q_matrix_pub(QMatrix) to publish the q-matrix

        # Create subscriber for reward of message type QLearningReward
        rospy.Subscriber("/q_learning/reward", QLearningReward, self.reward_received)
        
        # Create a publisher for robot actions
        self.robot_action_pub = rospy.Publisher("/q_learning/robot_action", RobotMoveDBToBlock, queue_size=1)

    def reward_received(data):
        reward = data.reward
        print("reward is " + str(reward))
        return reward

    def run(self):
        robot_action_data = RobotMoveDBToBlock()
        robot_action_data.robot_db = "red"
        robot_action_data.block_id = 1
        print("about to publish action")
        self.robot_action_pub.publish(robot_action_data)
        print("just published action")

        robot_action_data = RobotMoveDBToBlock()
        robot_action_data.robot_db = "green"
        robot_action_data.block_id = 2
        print("about to publish action")
        self.robot_action_pub.publish(robot_action_data)
        print("just published action")

        robot_action_data = RobotMoveDBToBlock()
        robot_action_data.robot_db = "blue"
        robot_action_data.block_id = 3
        print("about to publish action")
        self.robot_action_pub.publish(robot_action_data)
        print("just published action")

        rospy.spin()


if __name__=="__main__":

    node = Q_Learning()
    node.run()

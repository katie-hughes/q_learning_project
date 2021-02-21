#!/usr/bin/env python3

import rospy

from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from q_learning_project.msg import RobotMoveDBToBlock

import time

def find_state(red, green, blue):
    """ Simple way to assign all 64 states to a number.
        Assumes a dumbbell at the origin is assigned 0
        and a dumbbell at a block is assigned the block's number.
    """
    state_num = 1*red + 4*green + 16*blue
    return state_num


class QLearn(object):
    def __init__(self):
        rospy.init_node('q_learning_algorithm')
        #Q-Matrix:
        self.q = []
        #There are 8 actions:
        a = [0,0,0,0,0,0,0,0]
        for x in range(0, 64):
            self.q.append(a)
        for b in range(0, 4):
            for g in range(0, 4):
                for r in range(0, 4):
                    print("red:", r, "green:", g, "blue:", b, end='\t')
                    num = find_state(r,g,b)
                    print("num:", num)
    def run(self):
        rospy.spin()


if __name__=="__main__":

    node = QLearn()
    node.run()

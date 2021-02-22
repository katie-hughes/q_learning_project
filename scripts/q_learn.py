#!/usr/bin/env python3

import rospy

from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from q_learning_project.msg import RobotMoveDBToBlock

import time
import numpy as np
np.set_printoptions(threshold=np.inf)

def find_state(red, green, blue):
    """ Simple way to assign all 64 states to a number.
        Assumes a dumbbell at the origin is assigned 0
        and a dumbbell at a block is assigned the block's number.
    """
    state = 1*red + 4*green + 16*blue
    return state

def locations_from_state(state):
    """ Inverse of the 'find state' function
    """
    blue = state // 16
    green = (state - 16*blue) // 4
    red = state - 16*blue - 4*green
    return red, green, blue

def test_state():
    for b in range(0, 4):
        for g in range(0, 4):
            for r in range(0, 4):
                print("red:", r, "green:", g, "blue:", b, end='\t')
                num = find_state(r,g,b)
                print("State num:", num)
                red, green, blue = locations_from_state(num)
                print("red:", red, "green:", green, "blue:", blue, "\n")


def find_action(color, block):
    """ Assumes red=0, green=1, blue=2
        and the block is equal to its number
    """
    action = color*3+(block-1)
    return action

def inverse_action(action):
    """ Returns the color and block location from an action
    """
    color = action // 3
    block = action-color*3 + 1
    return color, block

def dumbbell_color(color):
    if color == 0:
        return "red"
    elif color == 1:
        return "green"
    elif color == 2:
        return "blue"
    else:
        return "UNKNOWN COLOR??"

def test_action():
    for color in range(0, 3):
        for block in range(1, 4):
            print("Moving the", dumbbell_color(color), "to block", block)
            action = find_action(color, block)
            print('Action number', action)
            c, b = inverse_action(action)
            print('Inverting: moving', dumbbell_color(c), "to", b, '\n')


class QLearn(object):
    def __init__(self):
        rospy.init_node('q_learning_algorithm')
        #Q-Matrix:
        self.q = []
        #There are 8 actions:
        a = [0,0,0,0,0,0,0,0]
        for x in range(0, 64):
            self.q.append(a)
        self.q = np.array(self.q)
        # creating the action matrix:
        self.actions = np.zeros((64,64), dtype=int)
        for si in range(0, 64):
            for sf in range(0, 64):
                # si is the initial state, f is the final state
                if si == sf:
                    # you cannot change into the same state
                    self.actions[si][sf] = -1
                    continue
                ri, gi, bi = locations_from_state(si)
                initial = np.array([ri, gi, bi])
                rf, gf, bf = locations_from_state(sf)
                final = np.array([rf, gf, bf])
                diff = final - initial
                if np.count_nonzero(diff) > 1:
                    #More than one dumbbell has moved
                    self.actions[si][sf] = -1
                    continue
                if np.count_nonzero(initial)>np.count_nonzero(final):
                    #A dumbbell got moved to the origin
                    self.actions[si][sf] = -1
                    continue
                #print(si, sf)
                #print('init:',initial)
                #print('final:', final)
                #print(diff)
                moved_dumbbell = np.nonzero(diff)[0][0] # the nonzero index of diff
                # 0 is red, 1 is green, 2 is blue
                #print(moved_dumbbell)
                source = initial[moved_dumbbell]
                destination = final[moved_dumbbell]
                #print(source, destination)
                required_action = find_action(moved_dumbbell, destination)
                #print(required_action)
                self.actions[si][sf] = required_action
                #print('\n')
        #test_state()
        #test_action()
        #print(self.actions)

    def run(self):
        rospy.spin()


if __name__=="__main__":

    node = QLearn()
    node.run()

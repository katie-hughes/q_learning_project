# q_learning_project

**Team: Elizabeth Singer and Adam Weider**

# Implementation plan

## Q-learning algorithm

### Executing the Q-learning algorithm

- Implementation: Set up the data structures for the Q-matrix and implement the
following tasks: initialize Q-matrix, select random action, perform random action,
receive reward for action, update Q-matrix, determine when Q-matrix has converged.
These will be implemented in a ROS node making use of the phantom robot node using
the sub/pub messages for determining the outcomes from the learning phase.

- Testing: A self-contained python node will be created to test each of the phases
of the Q-learning algorithm. Through printing states to the screen and running test
cases where the learning updates can be calculated by hand, the functions can be
tested and shown to be correct.

### Determining when the Q-matrix has converged

- Implementation: The convergence of the Q-matrix will be tested by checking the
Frobenius norm of the Q-matrix (sum of the squares of the matrix), or the Frobenius
norm of the change in the Q-matrix from one step to another (sum of the squares
of the delta from one update). Once the Frobenius norm ceases to change, or changes
by less than a threshold, the Q-matrix has converged.

- Testing: A simple loop can be created that changes a matrix by an amount that
decreases over time geometrically. Once the changes are small enough, the convergence
threshold should be met and the convergence can be tested. We can also print a
small subset of Q-matrix values to the screen to observe convergence.

### Choosing actions to maximize reward

- Implementation: Given the state of the world, the Q-matrix will show which
actions are feasible, and which have the highest expected future reward.  We can
simply select the one action of highest future reward, or randomly select actions
proportional to their expected future reward. If there are actions of equal future
reward, we can randomly select from among them.

- Testing: Given a Q-matrix, we can test that we select an appropriate action.
We can first do this with the phantom robot movement, and then once the movement
node is created, we can test with the turtlebot.

## Robot perception

### Identifying the colored dumbbells

- Implementation: We can move the pose of the robot to various positions and
detect the presence of a dumbbell using the given function. Using the color sensor
we can inspect the color returned and compare the colors to the color map for the
appropriate values for red, green and blue dumbbells.

- Testing: We can use Gazebo to place the robot in front of specific dumbbells of
specific colors and verify that the proper color is returned. For determining the
location of the dumbbells, exhaustive search would be one method for finding them.
We can use a known field setup in Gazebo to verify that our location and color
identification functions work.

### Identifying the numbered blocks

- Implementation: We can determine the locations of the blocks through a similar
method to determining the location of the dumbbells, that is searching each position
in the grid. Once in front of a block, we can use a python package for digit
recognition to determine which block is in place.

- Testing: We will import a digit recognition package from python from one of the
available packages, and test it on the sample blocks provided in Gazebo, assuming
that one isn’t provided in the class repository.

## Robot manipulation and movement

### Handling the dumbbells with the OpenMANIPULATOR arm

- Implementation: We will use the provided routines to command the manipulator
arm to the required angles to grip the dumbbells, and then to close the claw,
and then to lift the dumbbells. These required positions will be fixed since the
dumbbells are the same size and do not change. If the orientation of the robot
needs to be set properly in order to grab the dumbbells, we will implement a
proportional feedback routine to position the dumbbell in the center of the camera
so that it can be grabbed. The parameters of the positioning algorithm will need
to be learned from trial and error in Gazebo.

- Testing: Using Gazebo, we will place the dumbbells in front of the robot and
determine the necessary angles to command in order to address, grab, and lift the
dumbbells. We can similarly set up the dumbbells in front of the robot to test the
tracking algorithm to position the robot in place so that it can grab the dumbbells.


### Navigating to where the robot will handle the dumbbells

- Implementation: We can use the laser scanner and color scanner to determine the
location of the dumbbells in the world, and either determine the color of the
dumbbell at a distance (we can test this) or move to that location and use the
color sensor to determine which dumbbell is at that location. Once the dumbbell
locations are known, we can use the Q matrix to determine where to place them
using the manipulator functions from above.

- Testing: Testing plan: Using our movement framework, instruct the robot to
navigate to a specific dumbbell or block, and visually inspect the results. It
should be clear if something is amiss (e.g. navigating to an incorrect target,
missing the target entirely, colliding with the target, etc).

# Timeline of milestones

We plan to execute the q learning algorithm, determine when the q matrix has
converged, and determine which actions the robot should take to maximize expected
reward by February 20. We plan to determine the identities and locations of the
three colored dumbbells and determine the identities and locations of the three
numbered blocks by February 24. We plan to pick up and put down the dumbbells
with the OpenManipulator arm and navigate to the appropriate locations to pick
up and put down the dumb bells by February 28.

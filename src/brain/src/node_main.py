#!/usr/bin/python

###############################################################################
##                   University of California, Berkeley                      ##
##---------------------------------------------------------------------------##
##                  EECS 206A | Introduction to Robotics                     ##
##                          Tower-Building Baxter                            ##
##---------------------------------------------------------------------------##
##          Team: Rob Ruigrok, Gerrit Schoettler and Ruhullah Najafi         ##
###############################################################################
##                                                                           ##
##  This file contains the main classes we used for our game logic:          ##
##   -> TowerBuildingGame class                                              ##
##   -> Tower class                                                          ##
##   -> Block class                                                          ##
##                                                                           ##
###############################################################################

import rospy

import baxter_interface
from baxter_interface import CHECK_VERSION
from baxter_interface import gripper as robot_gripper

import moveit_commander
#from moveit_msgs.msg import OrientationConstraint, Constraints
#from geometry_msgs.msg import PoseStamped
from InverseKinematics import InverseKinematics

import sys
import time
import tf
#import numpy as np
from ar_track_alvar_msgs.msg import *
from std_msgs.msg import String, Int16MultiArray
from geometry_msgs.msg import Pose

from random import random
import math


ARM_TO_USE = 'left'
#ARM_TO_USE_MOVE_IT = 'left_arm'
#MOVE_IT_PLANNING_TIME = 10 # Amount of time the IK have to come up with a plan?

BLOCK_SIZE = 0.0508 # [m]

REF_BLOCK_ID = 17
BASE_BLOCK_ID = 15

SCAN_OFFSET_REF_IN_BLOCKS = 2 # [# of blocks]
SCAN_OFFSET_TOWER_IN_BLOCKS = 2 # [# of blocks]

PICK_UP_FIRST_SAFETY_HEIGHT = 0.10 # [m]
PICK_UP_SECOND_SAFETY_HEIGHT = 0.02 # [m]
PICK_UP_GRIPPING_OFFSET = -0.02 # [m]

# The relative height to the most upper block of the tower when moving blocks 
# towards the tower
PLACEMENT_MOVING_HEIGHT = 0.12 # [m]
PLACING_FIRST_SAFETY_HEIGHT = 0.03
PLACING_SECOND_SAFETY_HEIGHT = 0.01
PLACING_THIRD_SAFETY_HEIGHT = 0.006 #[m]
PLACING_RISK_FACTOR = 0.7  # between 0 and 1.0

PLAYING_ALONE = False

"""
    This class represents a block. 
    An instance of this class will contain the block's id, position and 
    orientation.
"""
class Block(object):

    def __init__(self, blockID, blockPosition = None, blockOrientation = None):
        """The constuctor of the Block class.
        Args:
            blockID (int):  The id of the new block.
            blockPosition (list(x,y,z)):        A list of x-y-z-values 
                                                representing the 3D position 
                                                of the new block.
            blockOrientation (list(x,y,z,w)):   A list of x-y-z-w-values 
                                                representing a quaternion for 
                                                the orientation of the new 
                                                block.
        Returns:
            Block:  A new instance of the class Block.
        """
        self.id = blockID
        self.position = blockPosition
        self.orientation = blockOrientation


"""
    This class represents a tower.
    A Tower object will include a dictionary of all the blocks making up the 
    tower. Furthermore it stores the id of the base block and the most upper 
    block. It also provides functions for adding blocks to it, finding out 
    whether a specific block is part of the tower and getting information about
    the base and most upper block.
"""
class Tower(object):

    def __init__(self):
        """The constuctor of the Tower class.
        Returns:
            Tower:  A new instance of the class Tower.
        """
        # A dictionary of all blocks making the current tower.
        # {blockID_1 : block_1, blockID_2 : block_2, ..., blockID_n : block_n}
        self.blocks = dict()

        self.baseBlockID = None # The id of the base block of the tower.
        self.topBlockID = None # The id of the most upper block of the tower.

    def isBlockPartOfTower(self, blockID):
        """Function to test whether a b specific block is part of the tower.
        Args:
            self (Tower):   A reference to the tower.
            blockID (int):  The id of the block to look for in the tower.
        Returns:
            bool:   True iff the block with the corresponding block id is part
                    of the tower.
        """
        return blockID in self.blocks.keys()

    def addBlock(self, blockID, blockPosition, blockOrientation):
        """Function to add a new block to the tower.
        Args:
            self (Tower):   A reference to the tower.
            blockID (int):  The id of the block which has to be added to the 
                            tower.
            blockPosition (list(x,y,z)):        A list of the x-y-z-position 
                                                of the block which has to be
                                                added to the tower.
            blockOrientation (list(x,y,z,w)):   A list of x-y-z-w-values 
                                                representing a quaternion for 
                                                the orientation of the block 
                                                which has to be added.
        """
        # Create a new block
        newBlock = Block(blockID, blockPosition, blockOrientation)
        self.topBlockID = blockID
        if self.baseBlockID == None:
            # If the tower was emty before
            self.baseBlockID = blockID
        # Add the new block to the dictionary of blocks
        self.blocks[blockID] = newBlock

    def getTopBlockPosition(self):
        """Function to get the position of the most upper block of the tower.
        Args:
            self (Tower):   A reference to the tower.
        Returns:
            list(): A list of x-y-z-values representing the 3D position of the 
                    most upper block of the tower.
        """
        if self.topBlockID != None:
            # If the tower already has a most upper block
            return self.blocks[self.topBlockID].position
        else:
            print("Tower.getTopBlockPosition(): The topBlockID is not set yet!")

    def getTopBlockOrientation(self):
        """Function to get the orientation of the most upper block of the tower.
        Args:
            self (Tower):   A reference to the tower.
        Returns:
            list(): A list of x-y-z-w-values representing a quaternion for the 
                    orientation of the most upper block of the tower.
        """
        if self.topBlockID != None:
            # If the tower already has a most upper block
            return self.blocks[self.topBlockID].orientation
        else:
            print("Tower.getTopBlockOrientation(): The topBlockID is not set yet!")

    def getBaseBlockPosition(self):
        """Function to get the position of the base block of the tower.
        Args:
            self (Tower):   A reference to the tower.
        Returns:
            list(): A list of x-y-z-values representing the 3D position of the 
                    base block of the tower.
        """
        if self.baseBlockID != None:
            # If the tower already has a base block
            return self.blocks[self.baseBlockID].position
        else:
            print("Tower.getBaseBlockPosition(): The baseBlockID is not set yet!")

    def getBaseBlockOrientation(self):
        """Function to get the orientation of the base block of the tower.
        Args:
            self (Tower):   A reference to the tower.
        Returns:
            list(): A list of x-y-z-w-values representing a quaternion for the 
                    orientation of the base block of the tower.
        """
        if self.baseBlockID != None:
            # If the tower already has a base block
            return self.blocks[self.baseBlockID].orientation
        else:
            print("Tower.getBaseBlockOrientation(): The baseBlockID is not set yet!")

    def reset(self):
        """Function to reset the tower.
        Args:
            self (Tower):   A reference to the tower.
        """
        self.blocks = dict()
        self.baseBlockID = None
        self.topBlockID = None

"""
    This class represents a game.
    A TowerBuildingGame object will contain all the necessary initializations 
    on the ROS side to be able to move the arm, handle the gripper and 
    calculate the necessary inverse kinematics. Furthermore it will inlcude 
    all the gamelogic itself.
"""
class TowerBuildingGame(object):

    def __init__(self):
        """The constuctor of the TowerBuildingGame class.
        Returns:
            TowerBuildingGame:  A new instance of the class TowerBuildingGame.
        """

        #####   ROS stuff   #####
        # Initializing AR_tag transforms
        self.listener = tf.TransformListener()

        # Verifying that robot is enabled
        self._robotState = baxter_interface.RobotEnable(CHECK_VERSION)
        self._initState = self._robotState.state().enabled
        self._robotState.enable()
        print("Robot is enabled.")

        # Initializing the arm and the gripper
        self.arm = baxter_interface.Limb(ARM_TO_USE)
        self.gripper = robot_gripper.Gripper(ARM_TO_USE)
        # Initializing the moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)

        #####   Game logic stuff   #####
        self.isRunning = True
        self.turn = 0 # Baxter: 0; Human: 1 (UNUSED AT THE MOMENT!)
        self.playAlone = False
        # Creating a Tower object which will represent the tower built
        self.tower = Tower()

        # Setting the default tower scanning position and orientation
        self.towerScanPosition = [0.7, 0.0, 0.0]
        self.towerScanOrientation = [0, -1, 0, 0]

        # Setting the default warehouse scanning position and orientation
        self.warehouseScanPosition = [0.7 ,0.3, 0.0]
        self.warehouseScanOrientation = [0, -1, 0, 0]

        # Creating placeholders for the measurement of the reference block 
        # after the first measurement and at the beginnging of every turn of 
        # Baxter
        self.refBlock_firstMeas = Block(REF_BLOCK_ID)
        self.refBlock = Block(REF_BLOCK_ID)

        # A placeholder for the block which Baxter decides to pick up next
        self.nextBlock = None

        print("Game instance is initialized successfully.")


    def moveArmToTowerScanPosition(self):
        """Function to move Baxter's arm to the tower scanning position.
        Args:
            self (TowerBuildingGame):   A reference to the game.
        """
        ik = InverseKinematics()
        if self.refBlock.position != None:
            # If it's not the first scan procedure of the tower
            mostUpperBlockPosition = self.tower.getTopBlockPosition()
            # To always stay a certain amount above the tower and reference 
            # point during the scanning procedure
            self.towerScanPosition[2] = max(
                mostUpperBlockPosition[2] + SCAN_OFFSET_TOWER_IN_BLOCKS * BLOCK_SIZE, 
                self.refBlock.position[2] + SCAN_OFFSET_REF_IN_BLOCKS * BLOCK_SIZE
            )
        ik.move(self.towerScanPosition, self.towerScanOrientation)

    def moveArmToWarehouseScanPosition(self):
        """Function to move Baxter's arm to the warehouse scanning position.
        Args:
            self (TowerBuildingGame):   A reference to the game.
        """
        ik = InverseKinematics()
        if self.tower.baseBlockID != None:
            # If there is already an existing tower, you want to make sure the
            # arm is moving high enough to not destroy the tower when moving 
            # away
            ik.move(
                self.warehouseScanPosition[:2] + [self.tower.getTopBlockPosition()[2] + 2.5 * BLOCK_SIZE], 
                self.warehouseScanOrientation
            )
        ik.move(self.warehouseScanPosition, self.warehouseScanOrientation)

    def scanTower(self):
        """Function to scan the tower (and the reference).
        Args:
            self (TowerBuildingGame):   A reference to the game.
        Returns:
            bool:   False iff something went wrong during the scanning 
                    procedure.
        """
        # Using the topic '/demo/found_markers' to get the marker IDs
        rospy.Subscriber(
            "/demo/found_markers", 
            Int16MultiArray, 
            foundMarkersCallback
        )
        msg1 = rospy.wait_for_message('/demo/found_markers', Pose)
        detectedMarkerIDs = msg1.data
        if len(detectedMarkerIDs) > 0:
            # Markers are detected
            try:
                if REF_BLOCK_ID in detectedMarkerIDs:
                    # Reference block found
                    targetMarker = 'ar_marker_'+str(REF_BLOCK_ID)
                    (pos, rot) = self.listener.lookupTransform(
                        '/base', 
                        targetMarker, 
                        rospy.Time(0)
                    )
                    self.refBlock.position = pos
                    self.refBlock.orientation = rot
                    if self.refBlock_firstMeas.position == None:
                        # First measurement of the reference
                        self.refBlock_firstMeas.position = pos
                        self.refBlock_firstMeas.orientation = rot
                    # Do NOT return True at this point, so you also look for 
                    # the blocks on the tower!!!
                else:
                    # Reference block not found
                    print("TowerBuildingGame.scanTower(): Reference block not found.")
                    return False

                if BASE_BLOCK_ID in detectedMarkerIDs:
                    # Base block found (tower just started to be built)
                    targetMarker = 'ar_marker_'+str(BASE_BLOCK_ID)
                    (pos, rot) = self.listener.lookupTransform(
                        '/base', 
                        targetMarker, 
                        rospy.Time(0)
                    )
                    # Reset the tower and add the base block to it
                    self.tower.reset()
                    self.tower.addBlock(BASE_BLOCK_ID, pos, rot)
                    return True
                elif len(self.tower.blocks) > 0:
                    # Base block not found, but the tower is already existing
                    # Excluse the reference in the set of potential most upper 
                    # blocks
                    potentialUpperBlocks = [block for block in detectedMarkerIDs if block != REF_BLOCK_ID]
                    for block in potentialUpperBlocks:
                        targetMarker = 'ar_marker_'+str(block)
                        (pos, rot) = self.listener.lookupTransform(
                            '/base', 
                            targetMarker, 
                            rospy.Time(0)
                        )
                        candidatePos = pos
                        baseBlockPos = self.tower.getBaseBlockPosition()
                        distanceToBaseBlock = math.hypot(
                            baseBlockPos[0] - candidatePos[0],
                            baseBlockPos[1] - candidatePos[1]
                        )
                        # Most upper block can only be in a radius of 1.2 
                        # blocks (approximately) around the centre of the base 
                        # base block
                        if distanceToBaseBlock <= 1.2 * BLOCK_SIZE:
                            # Most upper block of tower found
                            # Add to tower and return True
                            self.tower.addBlock(block, pos, rot)
                            return True
                    # If code reaches this point, then the most upper block 
                    # was not found
                    print("TowerBuildingGame.scanTower(): Most upper block not found.")
                    return False
                else:
                    # No current tower is existing and the base block was not 
                    # found
                    print("TowerBuildingGame.scanTower(): Base block not found.")
                    return False
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print("TowerBuildingGame.scanTower(): Failed to determine position and orientation of AR-Tag.")
                return False
        else:
            # No detected markers at all
            print("TowerBuildingGame.scanTower(): No AR-Tag found at tower scanning position.")
            return False

    def scanWarehouse(self, forNewBlock):
        """Function to scan the scanWarehouse.
        Args:
            self (TowerBuildingGame):   A reference to the game.
            forNextBlock (bool):        Tells whether we are looking for a new 
                                        block or are closing up on an already 
                                        chosen block.
        Returns:
            bool:   False iff something went wrong during the scanning 
                    procedure.
        """
        # Using the topic '/demo/found_markers' to get the marker IDs
        rospy.Subscriber(
            "/demo/found_markers", 
            Int16MultiArray, 
            foundMarkersCallback
        )
        msg1 = rospy.wait_for_message('/demo/found_markers', Pose)
        detectedMarkerIDs = msg1.data
        if len(detectedMarkerIDs) > 0:
            # Markers are detected
            try:
                if forNewBlock:
                    # Scanning for new nextBlock
                    # List with all detected ids which are not the reference id and not part of the current tower
                    potentialBlocks = [block for block in detectedMarkerIDs if block != REF_BLOCK_ID and (not self.tower.isBlockPartOfTower(block))]
                    if len(potentialBlocks) > 0:
                        # There are potential blocks to pick up
                        # Randomly choose one (here the first one is chosen)
                        chosenBlockID = potentialBlocks[0]
                        targetMarker = 'ar_marker_'+str(chosenBlockID)
                        (pos, rot) = self.listener.lookupTransform(
                            '/base', 
                            targetMarker, 
                            rospy.Time(0)
                        )
                        self.nextBlock = Block(chosenBlockID, pos, rot)
                        return True
                    else:
                        # No potential blocks to pick up
                        print("TowerBuildingGame.scanWarehouse(): No blocks found.")
                        return False
                elif self.nextBlock.id in detectedMarkerIDs:
                    # Scanning to update the pose(position and orientation) of nextBlock
                    targetMarker = 'ar_marker_'+str(self.nextBlock.id)
                    (pos, rot) = self.listener.lookupTransform(
                        '/base', 
                        targetMarker, 
                        rospy.Time(0)
                    )
                    self.nextBlock.position = pos
                    self.nextBlock.orientation = rot
                    return True
                else:
                    # During close up to the already chosen block it was lost
                    print("TowerBuildingGame.scanWarehouse(): Lost the block I wanted to pick up. Block_ID:" + str(self.nextBlock.id))
                    return False
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print("TowerBuildingGame.scanWarehouse(): Failed to determine position and orientation of AR-Tag.")
                return False
        else:
            # No detected markers at all
            print("TowerBuildingGame.scanWarehouse(): No AR-Tag found at warehouse scanning position.")
            return False

    def pickUpNextBlock(self):
        """Function to pick up the next block from the warehouse.
        Args:
            self (TowerBuildingGame):   A reference to the game.
        """
        ik = InverseKinematics()
        # First approach towards the target block
        ik.move(
            self.nextBlock.position[:2] + [self.nextBlock.position[2] + PICK_UP_FIRST_SAFETY_HEIGHT], 
            [0,-1,0,0]
        )
        # Update block position
        time.sleep(2)
        # The parameter false in self.scanWarehouse(bool) points out that we 
        # are closing up on an already chosen block to pick up
        while not self.scanWarehouse(False):
            # Scanning procedure of the chosen block failed
            # -> do it again...
            print("Scanning procedure will be executed again.")
            pass
        # Second approach towards the target block
        ik.move(
            self.nextBlock.position[:2] + [self.nextBlock.position[2] + PICK_UP_SECOND_SAFETY_HEIGHT], 
            [0,-1,0,0]
            #self.nextBlock.orientation
        )
        # Final approach to the target block
        self.arm.set_joint_position_speed(0.002)
        ik.move(
            self.nextBlock.position[:2] + [self.nextBlock.position[2] + PICK_UP_GRIPPING_OFFSET], 
            [0,-1,0,0]
            #self.nextBlock.orientation
        )
        self.arm.set_joint_position_speed(1.0)

    def moveUpBlock(self):
        """Function to move up the grabbed block from the warehouse.
        Args:
            self (TowerBuildingGame):   A reference to the game.
        """
        ik = InverseKinematics()
        ik.move(
            self.nextBlock.position[:2] + [self.tower.getTopBlockPosition()[2] + BLOCK_SIZE + PLACEMENT_MOVING_HEIGHT], 
            [0,-1,0,0]
        )

    def moveOverTover(self, riskFactor):
        """Function to move up the grabbed block over the tower.
        Args:
            self (TowerBuildingGame):   A reference to the game.
            riskFactor (float):         A factor saying how risky we want 
                                        baxter to place the block on top of 
                                        the tower.
        """
        ik = InverseKinematics()
        basePosInNewRef = [
            self.refBlock.position[0] + self.tower.getBaseBlockPosition()[0]-self.refBlock_firstMeas.position[0],
            self.refBlock.position[1] + self.tower.getBaseBlockPosition()[1]-self.refBlock_firstMeas.position[1]
        ]
        
        center_x = 0.5 * (basePosInNewRef[0] + self.tower.getTopBlockPosition()[0])
        center_y = 0.5 * (basePosInNewRef[1] + self.tower.getTopBlockPosition()[1])

        x_min = max(basePosInNewRef[0], self.tower.getTopBlockPosition()[0]) - BLOCK_SIZE/2
        x_max = min(basePosInNewRef[0], self.tower.getTopBlockPosition()[0]) + BLOCK_SIZE/2

        y_min = max(basePosInNewRef[1], self.tower.getTopBlockPosition()[1]) - BLOCK_SIZE/2
        y_max = min(basePosInNewRef[1], self.tower.getTopBlockPosition()[1]) + BLOCK_SIZE/2

        target_x = center_x + riskFactor * (-1 + 2*random()) * (x_max-center_x)
        target_y = center_y + riskFactor * (-1 + 2*random()) * (y_max-center_y)

        targetPosition = [target_x, target_y]

        ik.move(targetPosition + [self.tower.getTopBlockPosition()[2] + BLOCK_SIZE + PLACEMENT_MOVING_HEIGHT + PICK_UP_GRIPPING_OFFSET], [0,-1,0,0])
        self.arm.set_joint_position_speed(0.002)
        ik.move(targetPosition + [self.tower.getTopBlockPosition()[2] + BLOCK_SIZE + PLACING_FIRST_SAFETY_HEIGHT + PICK_UP_GRIPPING_OFFSET], [0,-1,0,0])
        #if raw_input("Want to go closer? (y) or (n)") == 'y':
        ik.move(targetPosition + [self.tower.getTopBlockPosition()[2] + BLOCK_SIZE + PLACING_SECOND_SAFETY_HEIGHT + PICK_UP_GRIPPING_OFFSET], [0,-1,0,0])
        #   if raw_input("Want to go even closer? (y) or (n)") == 'y':
        ik.move(targetPosition + [self.tower.getTopBlockPosition()[2] + BLOCK_SIZE + PLACING_THIRD_SAFETY_HEIGHT + PICK_UP_GRIPPING_OFFSET], [0,-1,0,0])
        self.nextBlock.position = targetPosition + [self.tower.getTopBlockPosition()[2] + BLOCK_SIZE]
        self.arm.set_joint_position_speed(1.0)
        
    def moveAwayFromTover(self):
        """Function to move away the gripper after placing the block on top of 
        the tower.
        Args:
            self (TowerBuildingGame):   A reference to the game.
        """
        ik = InverseKinematics()
        ik.move(
            self.tower.getTopBlockPosition()[:2] + [self.tower.getTopBlockPosition()[2] + 2.5 * BLOCK_SIZE], 
            [0,-1,0,0]
        )


def foundMarkersCallback(msg):
    # Do nothing here, because we get the information directly 
    # from the topic by aksing it
    pass

def main():
    game = TowerBuildingGame()
    game.gripper.calibrate()
    print("Arm calibrated.")

    if raw_input("Do you want the robot to play alone? (y) or (n)") == 'y':
        game.playAlone = True

    # Asking whether the scanning positions are fine
    print("Moving to default tower scanning position...")
    game.moveArmToTowerScanPosition()
    if raw_input('Do you want this as the default tower scanning position? (y) or (n)') == 'n':
        raw_input('Then move to your desired tower scanning position and press "Enter"')
        scanningPoint = game.arm.endpoint_pose()
        game.towerScanPosition = [scanningPoint['position'].x ,scanningPoint['position'].y, scanningPoint['position'].z]
        game.towerScanOrientation = [scanningPoint['orientation'].x ,scanningPoint['orientation'].y, scanningPoint['orientation'].z, scanningPoint['orientation'].w]
    
    print("Moving to default warehouse scanning position...")
    game.moveArmToWarehouseScanPosition()
    if raw_input('Do you want this as the default warehouse scanning position (y) or (n)') == 'n':
        raw_input('Then move to your desired warehouse scanning position and press "Enter"')
        scanningPoint = game.arm.endpoint_pose()
        game.warehouseScanPosition = [scanningPoint['position'].x ,scanningPoint['position'].y, scanningPoint['position'].z]
        game.warehouseScanOrientation = [scanningPoint['orientation'].x ,scanningPoint['orientation'].y, scanningPoint['orientation'].z, scanningPoint['orientation'].w]

    print("Main Game loop starts here.")
    while game.isRunning:
        game.moveArmToTowerScanPosition()
        # Sleep for a bit to let the arm settle down completely
        time.sleep(3)
        while not game.scanTower(): # Repeat until scan was successful
            # Scanning procedure of the warehouse failed
            # -> do it again...
            print("Scanning procedure will be executed again.")
            pass
        print("Scanning the tower was successful.")
        game.moveArmToWarehouseScanPosition()
        time.sleep(3)
        # The parameter false in self.scanWarehouse(bool) points out that we 
        # are closing up on an already chosen block to pick up
        while not game.scanWarehouse(True): # Repeat until scan was successful
            # Scanning procedure of the warehouse failed
            # -> do it again...
            print("Scanning the warehouse will be axecuted again.")
            pass
        print("Scanning the warehouse was successful.")
        game.pickUpNextBlock()
        time.sleep(1)
        game.gripper.close()
        time.sleep(0.3)
        game.moveUpBlock()
        game.moveOverTover(PLACING_RISK_FACTOR)
        time.sleep(1)
        game.gripper.open()
        time.sleep(0.3)
        game.moveAwayFromTover()
        game.tower.addBlock(game.nextBlock.id, game.nextBlock.position, game.nextBlock.orientation)
        game.nextBlock = None
        if not game.playAlone:
            if raw_input("It's your turn now, try to not disappoint me! Press (Enter) if you're done or (q) to quit the game.") == 'q':
                print("Thank you for the amaizing game! See you next time!")
                game.isRunning = False

if __name__ == "__main__":
    rospy.init_node('main_node')
    main()
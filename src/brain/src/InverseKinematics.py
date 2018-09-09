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
##  This InverseKinematics class listens for LaserScans and builds an        ##
##  occupancy grid.                                                          ##
##                                                                           ##
###############################################################################

import sys
import rospy
import moveit_commander
import string
from moveit_msgs.msg import OrientationConstraint, PositionConstraint, Constraints
from moveit_msgs.srv import GetPositionIKRequest
from geometry_msgs.msg import PoseStamped
#from intera_interface import gripper as robot_gripper

#import numpy as np
#from numpy import linalg

class InverseKinematics(object):

    def move(self, goalPosition, goalOrientation):

        left_arm = moveit_commander.MoveGroupCommander('left_arm')
        left_arm.set_planner_id('RRTConnectkConfigDefault')
        left_arm.set_planning_time(10)

        print("-> move()")
        request = GetPositionIKRequest()
        request.ik_request.group_name = "left_arm"
        request.ik_request.ik_link_name = "left_gripper"
        request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"
        
        #Set the desired orientation for the end effector HERE
        request.ik_request.pose_stamped.pose.position.x = goalPosition[0]
        request.ik_request.pose_stamped.pose.position.y = goalPosition[1]
        request.ik_request.pose_stamped.pose.position.z = goalPosition[2]
        request.ik_request.pose_stamped.pose.orientation.x = goalOrientation[0]
        request.ik_request.pose_stamped.pose.orientation.y = goalOrientation[1]
        request.ik_request.pose_stamped.pose.orientation.z = goalOrientation[2]
        request.ik_request.pose_stamped.pose.orientation.w = goalOrientation[3]
        
        try:
            goal_1 = PoseStamped()
            goal_1.header.frame_id = "base"

            #x, y, and z position
            goal_1.pose.position.x = goalPosition[0]
            goal_1.pose.position.y = goalPosition[1]
            goal_1.pose.position.z = goalPosition[2]
            
            #Orientation as a quaternion
            goal_1.pose.orientation.x = goalOrientation[0]
            goal_1.pose.orientation.y = goalOrientation[1]
            goal_1.pose.orientation.z = goalOrientation[2]
            goal_1.pose.orientation.w = goalOrientation[3]

            #Set the goal state to the pose you just defined
            left_arm.set_pose_target(goal_1)

            #Set the start state for the left arm
            left_arm.set_start_state_to_current_state()

            #Create a path constraint for the arm
            #UNCOMMENT TO ENABLE ORIENTATION CONSTRAINTS
            orien_const = OrientationConstraint()
            orien_const.link_name = "left_gripper";
            orien_const.header.frame_id = "base";
            orien_const.orientation.y = -1.0;
            orien_const.absolute_x_axis_tolerance = 0.1;
            orien_const.absolute_y_axis_tolerance = 0.1;
            orien_const.absolute_z_axis_tolerance = 0.1;
            orien_const.weight = 1.0;
            consts = Constraints()
            consts.orientation_constraints = [orien_const]

            # Attempt of position constraint:
            """
            pos_const = PositionConstraint()
            pos_const.link_name = "left_gripper";
            pos_const.header.frame_id = "base";
            #pos_const.position.z = goalPosition[2]; #.position does not exist
            pos_const.target_point_offset.z = [1.0,1.0,0]  # not workin
            pos_const.constraint_region = 0.1;
            pos_const.weight = 1.0;
            consts.position_constraints = [pos_const]
            """
            left_arm.set_path_constraints(consts)

            #Plan a path
            left_plan = left_arm.plan()
        
            #Execute the plan: 
            left_arm.execute(left_plan)
                
        except rospy.ServiceException, e:
            print "InverseKinematics: Service call failed: %s"%e
        print("<- move()")


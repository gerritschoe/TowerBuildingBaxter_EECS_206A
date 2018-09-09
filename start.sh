#!/bin/sh

# First of all make sure you have created a link to the baxter.sh shell script:
# ln -s /scratch/shared/baxter_ws/baxter.sh

# Connect to the robot:
# ./baxter.sh [robotname].local

#./baxter.sh asimov.local
#./baxter.sh archytas.local

echo "-> Starting awsome game ->"

echo "-> Start Camera setup done"
# Activate camera

echo "Closing all cameras...."
rosrun baxter_tools camera_control.py -c left_hand_camera
rosrun baxter_tools camera_control.py -c right_hand_camera
rosrun baxter_tools camera_control.py -c head_camera

echo "Turning on right hand camera..."
#rosrun baxter_tools camera_control.py -o right_hand_camera
rosrun baxter_tools camera_control.py -o left_hand_camera

echo "<- StartCamera setup done"

# Enable Baxter: 
#rosrun baxter_tools enable_robot.py -e

# Starting jointTrajectoryServer.sh

# Starting moveGroup.sh


echo "Starting main.launch"
roslaunch brain main.launch

#echo "<- Ending awsome game <-"

###############################################
# HOW TO SET UP THE SERVICES MANUALLY:

# close cameras and activate left hand camera
# rosrun baxter_tools enable_robot.py -e
# rosrun baxter_interface joint_trajectory_action_server.py
# roslaunch baxter_moveit_config move_group.launch			# this one works on archytas, not on Asimov
# roslaunch ar_track_alvar webcam_track.launch
# roslaunch brain main.launch

echo "Reached the End of the start.sh file"
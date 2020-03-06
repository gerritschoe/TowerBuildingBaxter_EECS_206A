# README #

## How to connect to baxter and start our code

### Connect to the robot:
./baxter.sh [robotname].local

### Close all cameras:
rosrun baxter_tools camera_control.py -c left_hand_camera
rosrun baxter_tools camera_control.py -c right_hand_camera
rosrun baxter_tools camera_control.py -c head_camera

### Turn on left hand camera:
rosrun baxter_tools camera_control.py -o left_hand_camera

### Enable Baxter: 
rosrun baxter_tools enable_robot.py -e

### Start joint_trajectory_action_server:
rosrun baxter_interface joint_trajectory_action_server.py

## New terminal, conencted to robot:
### Start move_group:
roslaunch baxter_moveit_config move_group.launch

## New terminal, conencted to robot:
### Start AR-Tracking:
roslaunch ar_track_alvar webcam_track.launch

## New terminal, conencted to robot:
### Start Main:
roslaunch brain main.launch

## New terminal, conencted to robot:
### Start Rviz:
rosrun rviz rviz

# What is this repository for? #

* This is the source code of our final project in 'EECS 206A | Introduction to Robotics' at the University of California, Berkeley in Fall 2017.
* Website: https://sites.google.com/berkeley.edu/towerbuildingbaxter-eecs206a/home

### Who do I talk to? ###

* Team: Gerrit Schoettler, Rob Ruigrok and Ruhullah Najafi
* Go to our website for more details.

# hrp-cv
hrp-cv is the source code used for a project in computer vision steering of a robotic lawn mower (Husqvarna research platform) using ROS.
## Requirements
All the code was developed for Ubuntu 16.04 and ROS Kinetic. <br>  
Besides this code, the Husqvarna research hrp source code (https://github.com/HusqvarnaResearch/hrp) was used to communicate with the robot.  <br>    
Since a raspicam was used for this project, the Ubiquity Robotics raspicam node (https://github.com/UbiquityRobotics/raspicam_node) was used as well.
## Packages
### cam-steer
The package contains two main nodes, *cam-steer* and *cam-controller*. 
#### cam-steer
The *cam-steer* node takes subscribes to the video stream of the raspicam, and publishes an estimated yaw angle of the robot. 
#### cam-controller
The *cam-controller* is a simple PID controller subscribing to the yaw angle from the *cam-steer* node and publishes control signals to the *cmd_vel* topic, which controls the robots velocities.

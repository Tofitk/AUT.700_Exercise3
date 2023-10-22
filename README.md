# AUT.700_Exercise3
Topi Kärki AUT.700 course exercise 3 version control/share 

This repo contains the ROS2 workspace task2_ws for the given exercise. task2_ws contains the ROS related things for tasks 2 and 3

Main python script for robot position management is found under the package task2_ws/src/robo_position_controller:
	
	move_to_N : these files contain the functionality of the controller individually. move_to_xytheta.py only has the ability to move to a point and to a specific angle. 
	
	robo_postion_controller_node.py: This contains all 3 wanted types of functionality in a single node.

Also contain some accesory files related to the exercise outside of task2_ws, like Task1 that has some items relating to that task

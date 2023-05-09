#! /usr/bin/env python3
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg


rospy.init_node('get_joint_status', anonymous=True)
robot = moveit_commander.RobotCommander()

#Get a RobotState message describing the current state of the robot
while not rospy.is_shutdown():
	# prints the values of the 4 joints + gripper +gropper_sub
	print ("Robot State: ", robot.get_current_state())
	rospy.sleep(3)


#! /usr/bin/env python3
import rospy
from std_msgs.msg import Float64, Float64MultiArray
setpoint = Float64()

def setpoint_callback(data):
    global setpoint
    setpoint.data = data.data[0]

rospy.init_node("setpoint_node")

setpoint_publisher = rospy.Publisher("/setpoint", Float64, queue_size=10)
rospy.Subscriber("/joint_angles", Float64MultiArray, setpoint_callback )

while not rospy.is_shutdown():

    setpoint_publisher.publish(setpoint)
    rospy.sleep(0.3)


import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import math

# Global list to store the joint angles
angles_msg = Float64MultiArray()
angles_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# Initializes the ROS node
rospy.init_node("joints_transform")

def map_to_degrees(x):
    """
    Maps a value x from the range (-1.57, 1.57) to the range (0, 180).
    """
    y = (x + math.pi/2) * 180 / math.pi
    y = max(0, min(180, y))
    return y

def gripper_state(x):
    """
    Maps a value x from the range (0, 0.3) to the range (180, 120).
    """
    y = 180 - x * 600
    y = max(120, min(180, y))
    return y

def callback(data:JointState):
    global angles_msg
    angles_msg.data = [
        map_to_degrees(data.position[0]),
        map_to_degrees(data.position[1]),
        map_to_degrees(data.position[2]),
        map_to_degrees(data.position[3]),
        gripper_state(data.position[4]),
        gripper_state(data.position[5])]

    angle_publisher.publish(angles_msg)

# Publish the angles_msg to the topic /joint_angles
angle_publisher = rospy.Publisher("/joint_angles", Float64MultiArray, queue_size=10)
rospy.Subscriber("/joint_states", JointState, callback)

rospy.spin()
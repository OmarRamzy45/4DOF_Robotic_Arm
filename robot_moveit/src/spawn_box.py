import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

rospy.wait_for_service('/gazebo/spawn_urdf_model')
spawn_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)

model_name = 'my_box'
model_xml = open('/home/ramzy/robot_urdf/src/robot_moveit/models/box.urdf', 'r').read()

reference_frame = 'robot_arm::base_link' # Replace with the reference frame of your robot
box_position = [1.0, 0.0, 0.5] # Define the position of the box relative to the reference frame of the robot
box_orientation = [0.0, 0.0, 0.0, 1.0] # Define the orientation of the box relative to the reference frame of the robot

initial_pose = Pose()
initial_pose.position.x = box_position[0]
initial_pose.position.y = box_position[1]
initial_pose.position.z = box_position[2]
initial_pose.orientation.x = box_orientation[0]
initial_pose.orientation.y = box_orientation[1]
initial_pose.orientation.z = box_orientation[2]
initial_pose.orientation.w = box_orientation[3]

spawn_model(model_name, model_xml, '', initial_pose, reference_frame)

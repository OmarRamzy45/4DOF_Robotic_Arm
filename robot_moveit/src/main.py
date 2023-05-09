#! /usr/bin/env python3
import sys
import copy
import rospy
import moveit_commander
import geometry_msgs.msg
import tf2_ros
import tf.transformations
import math
from std_msgs.msg import Float64, Bool


class Robot_Control:
    def __init__(self,node_name="robot_control",group_name="joint_group",planner_id="RRT",planning_time=1.0,num_planning_attempts=10,allow_replanning=True):
        '''
        constructor arguments:
            node_name: name of the node
            group_name: name of the group of joints to be controlled by moveit
            planner_id: name of the planner to be used by moveit
            planning_time: time allowed for planning
            num_planning_attempts: number of planning attempts
            allow_replanning: allow replanning
        '''
        # initialize the node
        rospy.init_node(node_name)

        # publisher for openning and closing the gripper
        self.gripper_pub = rospy.Publisher('/gripper_open_close', Float64, queue_size=10)

        # initialize moveit_commander and rospy node
        moveit_commander.roscpp_initialize(sys.argv)
        
        # Instantiate a RobotCommander object.
        # Provides information such as the robot’s kinematic model and the robot’s current joint states
        self.robot = moveit_commander.RobotCommander()
        print(self.robot.get_group_names())

        # Instantiate a PlanningSceneInterface object.
        # This provides a remote interface for getting, setting, and updating the robot’s internal understanding of the surrounding world:
        self.scene = moveit_commander.PlanningSceneInterface()

        # define the group of joints to be controlled by moveit
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        self.move_group.set_planner_id(planner_id)
        self.move_group.set_planning_time(planning_time)
        self.move_group.set_num_planning_attempts(num_planning_attempts)
        self.move_group.allow_replanning(allow_replanning)
        pass

    def control_joints_angles(self, joint_goal_list,velocity=0.1,acceleration=0.1,angle_is_degree=True):
        '''
        --------------------
        This function is used to move the robot to the desired joint state
        --------------------
        arguments:
            joint_goal: list of joint angles in degree
            velocity: velocity of the robot as a fraction of the maximum velocity
            acceleration: acceleration of the robot as a fraction of the maximum acceleration
            angle_is_degree : if the input angle is in degree
        '''
        if (angle_is_degree):
            joint_goal=[joint_goal_list[0]*math.pi/180,joint_goal_list[1]*math.pi/180,joint_goal_list[2]*math.pi/180,joint_goal_list[3]*math.pi/180]
        else:
            joint_goal=joint_goal_list
        # set the velocity of the robot
        self.move_group.set_max_velocity_scaling_factor(velocity)
        # set the acceleration of the robot
        self.move_group.set_max_acceleration_scaling_factor(acceleration)
        # set the goal joint state
        self.move_group.go(joint_goal,wait=True)
        # stop any residual movement
        self.move_group.stop()
        pass

    def move_to_pose(self, pose_name):
        '''
        --------------------
        This function is used to pick the object
        --------------------
        arguments:
            pose_name: name of the pose to be executed
        '''
        # set the target pose
        self.move_group.set_named_target(pose_name)
        print ("Executing Move:", pose_name)

        # plans a trajectory for the arm group
        plan = self.move_group.plan()

        # Use execute if you would like the robot to follow the plan that has already been computed:
        self.move_group.execute(plan[1], wait=True)

        # Calling ``stop()`` ensures that there is no residual movement group.stop()
        self.move_group.stop()

        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.move_group.clear_pose_targets()
        pass    
    
if __name__ == '__main__':
    try:
        robot = Robot_Control("robot_control","arm_group")
        gripper = Robot_Control("robot_control", "gripper_group")

        gripper.move_to_pose("open_gripper")
        # gripper.gripper_pub.publish(True)
        # robot.move_to_pose("pick")
        rospy.sleep(1)
        gripper.move_to_pose("close_gripper")
        # gripper.gripper_pub.publish(False)
        # robot.move_to_pose("idle")
        # robot.move_to_pose("place")
        # gripper.move_to_pose("open_gripper")
        # gripper.gripper_pub.publish(True)
        # robot.move_to_pose("idle")
        # gripper.move_to_pose("close_gripper")
        # gripper.gripper_pub.publish(False)

    except rospy.ROSInterruptException:
        pass

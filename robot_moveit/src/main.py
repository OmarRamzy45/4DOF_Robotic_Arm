#! /usr/bin/env python3
import sys
import copy
import rospy
import moveit_commander
import geometry_msgs.msg
import tf2_ros
import tf.transformations
import math
from std_msgs.msg import Float64, Bool, Char
from geometry_msgs.msg import Pose

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

        # initialize moveit_commander and rospy node
        moveit_commander.roscpp_initialize(sys.argv)
        
        # Instantiate a RobotCommander object.
        # Provides information such as the robot’s kinematic model and the robot’s current joint states
        self.robot = moveit_commander.RobotCommander()
        print(self.robot.get_group_names())

        # Instantiate a PlanningSceneInterface object.
        # This provides a remote interface for getting, setting, and updating the robot’s internal understanding of the surrounding world:
        self.scene = moveit_commander.PlanningSceneInterface()

        # subscriber to color_sensor topic
        self.color_sub = rospy.Subscriber("/color_sensor", Char, self.color_callback)

        # define the group of joints to be controlled by moveit
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        self.move_group.set_planner_id(planner_id)
        self.move_group.set_planning_time(planning_time)
        self.move_group.set_num_planning_attempts(num_planning_attempts)
        self.move_group.allow_replanning(allow_replanning)

        self.color = 'n'
        pass

    def color_callback(self, msg):
        '''
        --------------------
        This function is used to get the color from the color sensor
        --------------------
        arguments:
            msg: Char
        '''
        self.color = msg.data
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
        rospy.sleep(1)
        pass

    def get_pose(self):
        '''
        fuctionality:
            This function is used to get the robot's or the end effector
        --------------------
        arguments:
            no arguments
        --------------------
        This function is used to get the current pose of the robot
        --------------------
        return:
            pose: geometry_msgs.msg.Pose
        --------------------
        '''
        # get the current pose
        pose = self.move_group.get_current_pose().pose
        return 
        
    def go_to_pose_goal_cartesian(self, pose_goal,velocity=0.1,acceleration=0.1):
        '''
        --------------------
        This function is used to move the robot to the desired pose by cartesian path
        --------------------
        arguments:
            pose_goal: geometry_msgs.msg.Pose
            velocity: velocity of the robot
            acceleration: acceleration of the robot
        '''
        # set the velocity of the robot
        self.move_group.set_max_velocity_scaling_factor(velocity)
        # set the acceleration of the robot
        self.move_group.set_max_acceleration_scaling_factor(acceleration)
        # set the goal pose
        self.move_group.set_pose_target(pose_goal)
        # plan the motion
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        # clear the targets
        self.move_group.clear_pose_targets()
        pass

    def go_to_pose_goal_cartesian_waypoints(self, waypoints,velocity=0.1,acceleration=0.1,list_type=False):
        '''
        --------------------
        This function is used to move the robot to the desired pose by cartesian path
        --------------------
        arguments:
            if list_type is true 
                waypoints: nx6 list of waypoints
                    n: number of waypoints
                    6: x,y,z,roll,pitch,yaw
            if list_type is false 
                waypoints is given by type geometry_msgs.msg.Pose

            velocity: velocity of the robot
        
            acceleration: acceleration of the robot

            list_type indicatie if the given waypoints is geometry_msgs.msg.Pose or list
        '''
        geo_pose=geometry_msgs.msg.Pose() #create a geometry_msgs.msg.Pose() object
        list_of_poses = []
        for ways in waypoints:
            #set the position of the pose
            geo_pose.position.x = ways[0]
            geo_pose.position.y = ways[1]
            geo_pose.position.z = ways[2]
            #set the orientation of the pose
            quantrion = tf.transformations.quaternion_from_euler(ways[3],ways[4],ways[5])
            geo_pose.orientation.x = quantrion[0]
            geo_pose.orientation.y = quantrion[1]
            geo_pose.orientation.z = quantrion[2]
            geo_pose.orientation.w = quantrion[3]
            #append the pose to the list
            list_of_poses.append(copy.deepcopy(geo_pose))

        # set the goal pose
        (plan, fraction) = self.move_group.compute_cartesian_path(
                                    list_of_poses,   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0)         # jump_threshold
        # plan the motion

        # generate a new plan with the new velocity and acceleration by retiming the trajectory
        new_plan=self.move_group.retime_trajectory(self.robot.get_current_state(),plan,velocity_scaling_factor=velocity,acceleration_scaling_factor=acceleration)
        
        # execute the plan
        self.move_group.execute(new_plan,wait=True)

        pass
   
while not rospy.is_shutdown():
    try:
        robot = Robot_Control("robot_control","arm_group")
        gripper = Robot_Control("robot_control", "gripper_group")
                  
        robot.move_to_pose("idle")
        gripper.move_to_pose("open_gripper")
        robot.move_to_pose("pick_motor")
        robot.move_to_pose("pick")
        gripper.move_to_pose("close_gripper")
        robot.move_to_pose("pick_up")
        robot.move_to_pose("place_red_motor")
        robot.move_to_pose("place_red")
        gripper.move_to_pose("open_gripper")
        robot.move_to_pose("idle")

        robot.move_to_pose("pick_motor")
        robot.move_to_pose("pick")
        gripper.move_to_pose("close_gripper")
        robot.move_to_pose("pick_up")
        robot.move_to_pose("place_blue_motor")
        robot.move_to_pose("place_blue")
        gripper.move_to_pose("open_gripper")
        robot.move_to_pose("idle")

        robot.move_to_pose("pick_motor")
        robot.move_to_pose("pick")
        gripper.move_to_pose("close_gripper")
        robot.move_to_pose("pick_up")
        robot.move_to_pose("place_green_motor")
        robot.move_to_pose("place_green")
        gripper.move_to_pose("open_gripper")
        robot.move_to_pose("idle")
        gripper.move_to_pose("close_gripper")

    except rospy.ROSInterruptException:
        pass

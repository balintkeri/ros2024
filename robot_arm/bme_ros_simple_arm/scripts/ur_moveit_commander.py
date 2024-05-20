#!/usr/bin/env python3

import sys
import time
import rospy
import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list
import geometry_msgs.msg
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory

import inverse_kinematics
import detect_contacts
import attach
import detach

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupPythonInteface(object):
  """MoveGroupPythonInteface"""
  def __init__(self):

    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface', anonymous=True)

    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    ## for getting, setting, and updating the robot's internal understanding of the
    ## surrounding world:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints).  In this tutorial the group is the primary
    ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
    ## If you are using a different robot, change this value to the name of your robot
    ## arm planning group.
    ## This interface can be used to plan and execute motions:
    """
    group_names = robot.get_group_names()
    print("Available planning groups:")
    for name in group_names:
        print(name)
    """
    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    gripper_group = moveit_commander.MoveGroupCommander("gripper")

    # Create a publisher for the Gazebo simulated gripper
    gazebo_publisher = rospy.Publisher('/gripper_gazebo_controller/command', JointTrajectory, queue_size=1)

    # Getting Basic Information
    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print("============ Planning frame: %s" % planning_frame)

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print("============ End effector link: %s" % eef_link)

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print("============ Available Planning Groups:", robot.get_group_names())

    # Gazebo gripper
    self.gazebo_trajectory_command = JointTrajectory()
    self.gazebo_trajectory_command.joint_names = ["gripper"]
    self.gazebo_trajectory_point = JointTrajectoryPoint()
    self.gazebo_trajectory_point.time_from_start = rospy.rostime.Duration(1,0)
    self.gazebo_trajectory_point.velocities = [0.0]

    # Misc variables
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.gripper_group = gripper_group
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
    self.gazebo_publisher = gazebo_publisher

  def set_gripper(self, status):
      group_name = "gripper"
      #gripper_group = moveit_commander.MoveGroupCommander(group_name)
      joint_goal = self.gripper_group.get_current_joint_values()

      if status == "open":
        # Gazebo gripper value
        self.gazebo_trajectory_point.positions = [0.4]
        joint_goal[0] = 0.038
        joint_goal[1] = 0.038
      else:
        self.gazebo_trajectory_point.positions = [0.0]
        joint_goal[0] = 0.01
        joint_goal[1] = 0.01


      self.gripper_group.go(joint_goal, wait=True)

      # Calling ``stop()`` ensures that there is no residual movement
      self.gripper_group.stop()

      # For testing:
      current_joints = self.gripper_group.get_current_joint_values()
      return all_close(joint_goal, current_joints, 0.01)

      # Publish gazebo gripper position
      self.gazebo_trajectory_command.header.stamp = rospy.Time.now()
      self.gazebo_trajectory_command.points = [self.gazebo_trajectory_point]
      self.gazebo_publisher.publish(self.gazebo_trajectory_command)

  def go_to_joint_angles(self, joint_goals):

    ## Planning to a Joint Goal
    ## The UR's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
    ## thing we want to do is move it to a slightly better configuration.
    # We can get the joint values from the group and adjust some of the values:
    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[0] = joint_goals[0]
    joint_goal[1] = joint_goals[1]
    joint_goal[2] = joint_goals[2]
    joint_goal[3] = joint_goals[3]
    joint_goal[4] = joint_goals[4]
    joint_goal[5] = joint_goals[5]

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    self.move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    self.move_group.stop()

    # For testing:
    current_joints = self.move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def go_to_pose(self, x, y, z):
    ## Planning to a Pose Goal
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    pose_goal = geometry_msgs.msg.Pose()
    # set proper quaternion for the vertical orientation: https://quaternions.online/
    #pose_goal.orientation.x = 0.383 # -1
    #pose_goal.orientation.y = -0.4

    
    pose_goal.orientation.x = 0
    pose_goal.orientation.y = 0
    pose_goal.orientation.z = 0
    pose_goal.orientation.w = 1
    
    
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z

    ###
    current_pose = self.move_group.get_current_pose().pose
    print("############")
    print(current_pose)

    self.move_group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = self.move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    self.move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.move_group.clear_pose_targets()

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.move_group.get_current_pose().pose
    #print("END POSE:")
    #print(current_pose)
    return all_close(pose_goal, current_pose, 0.01)

  def go_to_named_target(self, name):
    self.move_group.set_named_target(name)

    ## Now, we call the planner to compute the plan and execute it.
    plan = self.move_group.go(wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    self.move_group.stop()
  
  def get_joint_values(self):
    current_join_values = self.move_group.get_current_joint_values()
    print("JOINT VALUES")
    print(current_join_values)
    return current_join_values

  def set_wrist_joint(self, joint_values):
    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[3] = joint_values[3]
    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    self.move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    self.move_group.stop()

    # For testing:
    current_joints = self.move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

def main():
  try:

    moveit_commander = MoveGroupPythonInteface()

    # Set max velocity
    moveit_commander.move_group.set_max_velocity_scaling_factor(0.2)
    # Set tolerances, without that IK cannot do a valid plan
    moveit_commander.move_group.set_goal_position_tolerance(0.05)
    moveit_commander.move_group.set_goal_orientation_tolerance(0.1)

    time.sleep(2)
    place_dict = {"red": [0.5, 0, 0.065], "green": [0.4, -0.2,  0.065]}

    for key in place_dict:
    #joint_values = moveit_commander.get_joint_values()

      joint_angles = inverse_kinematics.inverse_kinematics(place_dict[key], "open", 0)
      print("###JOINT ANGLES###")
      print(joint_angles)
      moveit_commander.set_gripper("open")

      input("============ Press `Enter` to go to X,Y,Z coordinates...")
      moveit_commander.go_to_joint_angles(joint_angles)

      input("============ Press `Enter` to close gripper...")
      moveit_commander.set_gripper("closed")

      if(detect_contacts.check_for_contact()):
        print("### Contact Detected ###")
        attach.attach_left_finger("red box")
      
      moveit_commander.go_to_named_target("home")

      ###the target position
      joint_angles = inverse_kinematics.inverse_kinematics([0.00001, -0.5, 0.05], "open", 0)
      print("###JOINT ANGLES###")
      print(joint_angles)
      input("============ Press `Enter` to go to X,Y,Z coordinates...")
      moveit_commander.go_to_joint_angles(joint_angles)
      detach.detach_left_finger("red box")
      joint_angles = inverse_kinematics.inverse_kinematics([0.00001, -0.5, 0.15], "open", 0)
      moveit_commander.go_to_joint_angles(joint_angles)

      input("============ Press `Enter` to open gripper and go to home...")
      moveit_commander.set_gripper("open")
      moveit_commander.go_to_named_target("home")


    """
    input("============ Press `Enter` to go joint angles...")
    #moveit_commander.go_to_joint_angles([-1.5708, -1.5708, -1.0472, -1.0472, 1.5708, 0.7854])
    moveit_commander.go_to_joint_angles([-1.5708, -1.5708, -1.0472])

    input("============ Press `Enter` to close gripper...")
    moveit_commander.set_gripper("closed")

    input("============ Press `Enter` to go to X,Y,Z coordinates...")
    moveit_commander.go_to_pose(0.22, 0.0022, 0.463)
    
    input("============ Press `Enter` to open gripper...")
    moveit_commander.set_gripper("open")
    
    input("============ Press `Enter` to go up position...")
    moveit_commander.go_to_named_target("home")
    """

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
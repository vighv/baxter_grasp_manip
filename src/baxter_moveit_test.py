#!/usr/bin/env python

import argparse
import sys

import copy

import rospy

import actionlib

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

import baxter_interface

from baxter_interface import CHECK_VERSION

import numpy as np

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

#from baxter_core_msgs import EndPointState

from std_msgs.msg import String

def move_group_python_interface_tutorial():
  ## BEGIN_TUTORIAL
  ##
  ## Setup
  ## ^^^^^
  ## CALL_SUB_TUTORIAL imports
  ##
  ## First initialize moveit_commander and rospy.
  print "============ Starting tutorial setup"
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface_tutorial',
                  anonymous=True)

  ## Instantiate a RobotCommander object.  This object is an interface to
  ## the robot as a whole.
  robot = moveit_commander.RobotCommander()

  ## Instantiate a PlanningSceneInterface object.  This object is an interface
  ## to the world surrounding the robot.
  scene = moveit_commander.PlanningSceneInterface()

  ## Instantiate a MoveGroupCommander object.  This object is an interface
  ## to one group of joints.  In this case the group is the joints in the left
  ## arm.  This interface can be used to plan and execute motions on the left
  ## arm.
  group = moveit_commander.MoveGroupCommander("left_arm")

  #group.set_planner_id("RRTConnectkConfigDefault")
  group.set_planner_id("RRTStarkConfigDefault")
  #group.set_planner_id("LBKPIECEkConfigDefault")
  group.set_planning_time(30)
  group.set_num_planning_attempts(30)
  group.set_goal_position_tolerance(0.05)
  group.set_goal_orientation_tolerance(0.05)
  group.allow_replanning(True)
  group.allow_looking(True)
  #group.set_max_velocity_scaling_factor(0.3)
  ## We create this DisplayTrajectory publisher which is used below to publish
  ## trajectories for RVIZ to visualize.
  display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory, queue_size=20)

  ## Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
  print "============ Waiting for RVIZ..."
  rospy.sleep(2)
  print "============ Starting tutorial "

  ## Getting Basic Information
  ## ^^^^^^^^^^^^^^^^^^^^^^^^^
  ##
  ## We can get the name of the reference frame for this robot
  print "============ Reference frame: %s" % group.get_planning_frame()

  ## We can also print the name of the end-effector link for this group
  print "============ Reference frame: %s" % group.get_end_effector_link()

  ## We can get a list of all the groups in the robot
  print "============ Robot Groups:"
  print robot.get_group_names()

  ## Sometimes for debugging it is useful to print the entire state of the
  ## robot.
  print "============ Printing robot state"
  print robot.get_current_state()
  print "============"


  ## Planning to a Pose goal
  ## ^^^^^^^^^^^^^^^^^^^^^^^
  ## We can plan a motion for this group to a desired pose for the 
  ## end-effector
  print "============ Generating plan 1"
  pose_target = geometry_msgs.msg.Pose()
  current_pose = group.get_current_pose()
  print current_pose
  #pose_target.orientation.w = 1.0
  pose_target.orientation = current_pose.pose.orientation
  pose_target.position.x = current_pose.pose.position.x + 0.1
  pose_target.position.y = current_pose.pose.position.y + 0.1
  pose_target.position.z = current_pose.pose.position.z + 0.1
  #group.set_position_target(np.array([pose_target.position.x, pose_target.position.y, pose_target.position.z]))
  group.set_pose_target(pose_target)


  # ## Now, we call the planner to compute the plan
  # ## and visualize it if successful
  # ## Note that we are just planning, not asking move_group 
  # ## to actually move the robot
  plan1 = group.plan()

  print "============ Waiting while RVIZ displays plan1..."
  rospy.sleep(5)

 
  # ## You can ask RVIZ to visualize a plan (aka trajectory) for you.  But the
  # ## group.plan() method does this automatically so this is not that useful
  # ## here (it just displays the same trajectory again).
  print "============ Visualizing plan1"
  # display_trajectory = moveit_msgs.msg.DisplayTrajectory()

  # display_trajectory.trajectory_start = robot.get_current_state()
  # display_trajectory.trajectory.append(plan1)
  # display_trajectory_publisher.publish(display_trajectory);

  # print "============ Waiting while plan1 is visualized (again)..."
  # rospy.sleep(5)


  ## Moving to a pose goal
  ## ^^^^^^^^^^^^^^^^^^^^^
  ##
  ## Moving to a pose goal is similar to the step above
  ## except we now use the go() function. Note that
  ## the pose goal we had set earlier is still active 
  ## and so the robot will try to move to that goal. We will
  ## not use that function in this tutorial since it is 
  ## a blocking function and requires a controller to be active
  ## and report success on execution of a trajectory.

  # Uncomment below line when working with a real robot
  # print "============ Going to goal pose on real robot..."
  # group.go(wait=True)

  ## Planning to a joint-space goal 
  ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  ##
  ## Let's set a joint space goal and move towards it. 
  ## First, we will clear the pose target we had just set.

  # group.clear_pose_targets()

  ## Then, we will get the current set of joint values for the group
  # group_variable_values = group.get_current_joint_values()
  # print "============ Joint values: ", group_variable_values

  # ## Now, let's modify one of the joints, plan to the new joint
  # ## space goal and visualize the plan
  # group_variable_values[0] = 0.3
  # group.set_joint_value_target(group_variable_values)

  # plan2 = group.plan()

  # print "============ Waiting while RVIZ displays plan2..."
  # rospy.sleep(5)


  ## Cartesian Paths
  ## ^^^^^^^^^^^^^^^
  ## You can plan a cartesian path directly by specifying a list of waypoints 
  ## for the end-effector to go through.
  # waypoints = []

  # # start with the current pose
  # waypoints.append(group.get_current_pose().pose)

  # # first orient gripper and move forward (+x)
  # wpose = geometry_msgs.msg.Pose()
  # wpose.orientation.w = 1.0
  # wpose.position.x = waypoints[0].position.x + 0.1
  # wpose.position.y = waypoints[0].position.y
  # wpose.position.z = waypoints[0].position.z
  # waypoints.append(copy.deepcopy(wpose))

  # # second move down
  # wpose.position.z -= 0.10
  # waypoints.append(copy.deepcopy(wpose))

  # # third move to the side
  # wpose.position.y += 0.05
  # waypoints.append(copy.deepcopy(wpose))

  # ## We want the cartesian path to be interpolated at a resolution of 1 cm
  # ## which is why we will specify 0.01 as the eef_step in cartesian
  # ## translation.  We will specify the jump threshold as 0.0, effectively
  # ## disabling it.
  # (plan3, fraction) = group.compute_cartesian_path(
  #                              waypoints,   # waypoints to follow
  #                              0.01,        # eef_step
  #                              0.0)         # jump_threshold
                               
  # print "============ Waiting while RVIZ displays plan3..."
  # rospy.sleep(5)

 
  # ## Adding/Removing Objects and Attaching/Detaching Objects
  # ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  # ## First, we will define the collision object message
  # collision_object = moveit_msgs.msg.CollisionObject()
  print "============ Going to goal pose on real robot..."
  group.go(wait=True)


  ## When finished shut down moveit_commander.
  moveit_commander.roscpp_shutdown()

  ## END_TUTORIAL

  print "============ STOPPING"


if __name__=='__main__':
  try:
    move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
	pass
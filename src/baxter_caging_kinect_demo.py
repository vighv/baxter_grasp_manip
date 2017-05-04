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
import baxter_external_devices

from baxter_interface import CHECK_VERSION

import numpy as np

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Vector3, Twist
from pyquaternion import Quaternion

#from baxter_core_msgs import EndPointState

from std_msgs.msg import String

class BaxterGraspKinect():
  
  def __init__(self):
    rospy.init_node("baxter_grasp_kinect")

    self.rod_end1 = Vector3(0,0,0)
    self.rod_end2 = Vector3(0,0,0)
    self.diameter = 33.5/1000.0 #33.5 mm diameter of rod

    self.flag1 = 0
    self.flag2 = 0

  def end1_callback(self,msg):
    self.rod_end1 = Vector3(msg.x,msg.y,msg.z)
    print "============ Rod end point 1 in Kinect frame"
    print self.rod_end1 
    self.flag1 = 1
    #subscriber.unregister()

  def end2_callback(self,msg):
    self.rod_end2 = Vector3(msg.x,msg.y,msg.z)
    print "============ Rod end point 2 in Kinect frame"
    print self.rod_end2
    self.flag2 = 1
    #subscriber.unregister()

  def get_kinect(self):

    self.rod_end1 = rospy.wait_for_message('/pos_1',Vector3)
    self.rod_end2 = rospy.wait_for_message('/pos_2',Vector3)
 
    print "============ Rod end point 1 in Kinect frame"
    print self.rod_end1 

    print "============ Rod end point 2 in Kinect frame"
    print self.rod_end2 

    self.rod_mid_point_kinect = Vector3( 0.5*(self.rod_end1.x + self.rod_end2.x) , 0.5*((self.rod_end1.y + self.rod_end2.y)) , 0.5*((self.rod_end1.z + self.rod_end2.z))  )
    
    print "============ Rod mid point in Kinect frame"
    print self.rod_mid_point_kinect
    # self.rate = rospy.Rate(10)

    # while not self.flag1*self.flag2:            
    #   rospy.Subscriber('/pos_1', Vector3, self.end1_callback)            
    #   rospy.Subscriber('/pos_2', Vector3, self.end2_callback)
      
    #   ## Check for Kinect tracking of end points of the rod
    #   rospy.wait_for_message('/pos_1', Vector3)
    #   rospy.wait_for_message('/pos_2', Vector3) 
    # self.rate.sleep()
  
  def transform_kinect_to_baxter(self):

    # Transform parameters from Baxter to Kinect
    xb = -0.735
    yb = -0.032
    zb = -1.2591
    self.vec_baxter_to_kinect = np.array([xb , yb, zb])


    matrix_se3 = np.array([ [0,1,0,-xb] , [1,0,0,-yb] , [0,0,-1,-zb] , [0,0,0,1] ])

    v1 = np.array([self.rod_end1.x , self.rod_end1.y , self.rod_end1.z , 1])
    self.v1_bax = matrix_se3.dot(v1) 
    self.rod_end1_baxter = Vector3(self.v1_bax[0] , self.v1_bax[1] , self.v1_bax[2])

    print "============ Rod end point 1 in Baxter frame"
    print self.rod_end1_baxter 

    v2 = np.array([self.rod_end2.x , self.rod_end2.y , self.rod_end2.z , 1])
    self.v2_bax = matrix_se3.dot(v2) 
    self.rod_end2_baxter = Vector3(self.v2_bax[0] , self.v2_bax[1] , self.v2_bax[2])
 

    print "============ Rod end point 2 in Baxter frame"
    print self.rod_end2_baxter

    # proceed = raw_input("Do you want to continue with moving Baxter (y/n): ")
    
    # if proceed in ['y' , 'Y']:
    #   self.move_baxter_grasp()
    # else:
    #   exit()
 

  def move_baxter_grasp(self):
    ## BEGIN_TUTORIAL
    ##
    ## Setup
    ## ^^^^^
    ## CALL_SUB_TUTORIAL imports
    ##
    ## First initialize moveit_commander and rospy.
    
    self.rod_mid_point = Vector3( 0.5*(self.v1_bax[0] + self.v2_bax[0]) , 0.5*(self.v1_bax[1] + self.v2_bax[1]) , 0.5*(self.v1_bax[2] + self.v2_bax[2])  )
    
    print "============ Rod mid point in Baxter frame"
    print self.rod_mid_point 

    print "============ Starting Moveit Commander"
    moveit_commander.roscpp_initialize(sys.argv)

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
    group_gripper = moveit_commander.MoveGroupCommander("left_hand")

    #group.set_planner_id("RRTConnectkConfigDefault")
    group.set_planner_id("RRTStarkConfigDefault")
    #group.set_planner_id("LBKPIECEkConfigDefault")
    group.set_planning_time(20)
    group.set_num_planning_attempts(30)
    group.set_goal_position_tolerance(0.01)
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
    # pose_target.position.x = current_pose.pose.position.x + 0.1
    # pose_target.position.y = current_pose.pose.position.y + 0.1
    # pose_target.position.z = current_pose.pose.position.z + 0.1
    pose_target.position.x = self.rod_mid_point.x
    pose_target.position.y = self.rod_mid_point.y
    pose_target.position.z = -0.05    

    #group.set_position_target(np.array([pose_target.position.x, pose_target.position.y, pose_target.position.z]))
    group.set_pose_target(pose_target)


    plan1 = group.plan()

    print "============ Waiting while RVIZ displays plan1..."
    rospy.sleep(2)

   
    # ## You can ask RVIZ to visualize a plan (aka trajectory) for you.  But the
    # ## group.plan() method does this automatically so this is not that useful
    # ## here (it just displays the same trajectory again).
    

    print "============ Going to goal pose on real robot..."
    group.go(wait=True)
    
    ## When finished shut down moveit_commander.
    


    print "============ Grasping Maneuvers: Rotating gripper to align with rod"
    
        # Planning to a joint-space goal 
    # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    #
    # Let's set a joint space goal and move towards it. 
    # First, we will clear the pose target we had just set.

    group.clear_pose_targets()

    # Then, we will get the current set of joint values for the group
    current_pose = group.get_current_pose()
    pose_target.position = current_pose.pose.position

    rot_angle = np.arctan2(self.rod_end2_baxter.y - self.rod_end1_baxter.y , self.rod_end2_baxter.x - self.rod_end1_baxter.x)

    rot_quaternion = Quaternion(axis=[1, 0, 0] , angle=rot_angle)
    print "Rotation Quaternion ", rot_quaternion
    quat_vec = rot_quaternion.elements

    pose_target.orientation.x = quat_vec[0]
    pose_target.orientation.y = quat_vec[1]
    pose_target.orientation.z = quat_vec[2]
    pose_target.orientation.w = quat_vec[3]
    
    print "Pose target ",pose_target
    
    group.set_pose_target(pose_target)
    # print "============ Joint values: ", group_variable_values

    # ## Now, let's modify one of the joints, plan to the new joint
    # ## space goal and visualize the plan
    # group_variable_values[6] = 0.0
    # group.set_joint_value_target(group_variable_values)



    plan2 = group.plan()

    print "============ Waiting while RVIZ displays plan2..."
    rospy.sleep(2)
    group.go(wait=True)

    
    ## Now to perform grasping:
    # done = False

    # left = baxter_interface.Limb('left')
    
    # lj = left.joint_names()
    # print lj
    # curr_l_w2 = left.joint_angle(lj[6])
    # print curr_l_w2
    # command_l_w2 = {lj[6] : 0.0}

    #left.set_joint_positions(command_l_w2)
    # left.move_to_joint_positions(command_l_w2)
    #grip_left.close()
    
    lower_flag = raw_input('Safe to go down to grasp? [y/n]: ')

    if lower_flag in ['y' , 'Y']:
      group.clear_pose_targets()
      # Then, we will get the current set of joint values for the group
      current_pose = group.get_current_pose()
      pose_target.orientation = current_pose.pose.orientation
      pose_target.position = current_pose.pose.position
      pose_target.position.z = -0.12
      
      print "Pose target for holding object ",pose_target
      
      group.set_pose_target(pose_target)
    # print "============ Joint values: ", group_variable_values

    # ## Now, let's modify one of the joints, plan to the new joint
    # ## space goal and visualize the plan
    # group_variable_values[6] = 0.0
    # group.set_joint_value_target(group_variable_values)

      plan3 = group.plan()
      group.go(wait=True)

      print "============ Moving while caged"
      group.clear_pose_targets()
      # Then, we will get the current set of joint values for the group
      current_pose = group.get_current_pose()
      pose_target.orientation = current_pose.pose.orientation
      pose_target.position.x = current_pose.pose.position.x + 0.05
      pose_target.position.y = current_pose.pose.position.y - 0.1
      pose_target.position.z = -0.12
      
      #print "Pose target for holding object ",pose_target
      
      group.set_pose_target(pose_target)
    # print "============ Joint values: ", group_variable_values

    # ## Now, let's modify one of the joints, plan to the new joint
    # ## space goal and visualize the plan
    # group_variable_values[6] = 0.0
    # group.set_joint_value_target(group_variable_values)

      plan5 = group.plan()
      group.go(wait=True)

    # print "============ Gripper Move Group Current JVs "  
    current_state_gripper_jts = group_gripper.get_joints()
    current_state_gripper = group_gripper.get_active_joints()
    print current_state_gripper

    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    gripper_left = baxter_interface.Gripper('left', CHECK_VERSION)
    rospy.sleep(3)
    gripper_left.close()


    grip_flag = raw_input('Enter any key after running gripper subroutine: ')
    #moveit_commander.roscpp_shutdown()

    # grip_left = baxter_interface.Gripper('left')
    # grip_left.open()
    # rospy.sleep(2)
    # grip_left.close()

    # rospy.sleep(3)

    # moveit_commander.roscpp_initialize(sys.argv)
    # rospy.sleep(5)
    # ## Instantiate a RobotCommander object.  This object is an interface to
    # ## the robot as a whole.
    # robot = moveit_commander.RobotCommander()

    # ## Instantiate a PlanningSceneInterface object.  This object is an interface
    # ## to the world surrounding the robot.
    # scene = moveit_commander.PlanningSceneInterface()

    # ## Instantiate a MoveGroupCommander object.  This object is an interface
    # ## to one group of joints.  In this case the group is the joints in the left
    # ## arm.  This interface can be used to plan and execute motions on the left
    # ## arm.
    # group = moveit_commander.MoveGroupCommander("left_arm")
    # group_gripper = moveit_commander.MoveGroupCommander("left_hand")

    # group.set_planner_id("RRTConnectkConfigDefault")
    # ##group.set_planner_id("RRTStarkConfigDefault")
    # # #group.set_planner_id("LBKPIECEkConfigDefault")
    # group.set_planning_time(10)
    # group.set_num_planning_attempts(30)
    # group.set_goal_position_tolerance(0.01)
    # group.set_goal_orientation_tolerance(0.05)
    # group.allow_replanning(True)
    # group.allow_looking(True)
    # #group.set_max_velocity_scaling_factor(0.3)
    # # ## We create this DisplayTrajectory publisher which is used below to publish
    # # ## trajectories for RVIZ to visualize.
    # display_trajectory_publisher = rospy.Publisher(
    #                                     '/move_group/display_planned_path',
    #                                     moveit_msgs.msg.DisplayTrajectory, queue_size=20)

    # ## END_TUTORIAL

    print "============ Grasping And Moving: Straight Up"
    
    group.clear_pose_targets()

    # Then, we will get the current set of joint values for the group
    current_pose = group.get_current_pose()
    pose_target.position = current_pose.pose.position
    pose_target.position.z = -0.04
    pose_target.orientation = current_pose.pose.orientation 
    
    group.set_pose_target(pose_target)
    plan4 = group.plan()

    group.go(wait=True)

    print "============ Moving to Goal"
    
    group.clear_pose_targets()

    # Then, we will get the current set of joint values for the group
    current_pose = group.get_current_pose()
    pose_target.position.x = 0.744
    pose_target.position.y = -0.084
    pose_target.position.z = 0.04
    pose_target.orientation = current_pose.pose.orientation
    # pose_target.orientation.x = 1.00
    # pose_target.orientation.y = 0.00
    # pose_target.orientation.z = 0.00
    # pose_target.orientation.w = 0.00
    
    #group.set_pose_target(pose_target)
    group.set_position_target([pose_target.position.x , pose_target.position.y, pose_target.position.z])
    plan6 = group.plan()

    group.go(wait=True)

    moveit_commander.roscpp_shutdown()

    grip_left = baxter_interface.Gripper('left')
    grip_left.open()
    rospy.sleep(2)
    # grip_left.close()
    print "============ STOPPING"


if __name__=='__main__':
  
  try:
    baxter_test = BaxterGraspKinect()
    baxter_test.get_kinect()
    baxter_test.transform_kinect_to_baxter()
    baxter_test.move_baxter_grasp()


  except rospy.ROSInterruptException:
	pass
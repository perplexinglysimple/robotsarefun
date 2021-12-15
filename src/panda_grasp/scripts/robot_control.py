#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from tf.transformations import quaternion_from_euler
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from sensor_msgs.msg import JointState
import threading


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


class DoublePandaControl(object):
  """DoublePandaControl"""
  def __init__(self):
    super(DoublePandaControl, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('robot_control', anonymous=True)

    robot = moveit_commander.RobotCommander()

    #scene = moveit_commander.PlanningSceneInterface()

    arm_1_group = "panda_1_arm"
    hand_1_group = "hand_1"
    move_group_1 = moveit_commander.MoveGroupCommander(arm_1_group)
    hand_1 = moveit_commander.MoveGroupCommander(hand_1_group)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    planning_frame_1 = move_group_1.get_planning_frame()

    eef1_link = move_group_1.get_end_effector_link()

    group_names = robot.get_group_names()

    self.box_name = ''
    self.robot = robot
    #self.scene = scene
    self.move_group_1 = move_group_1
    self.hand_1 = hand_1
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame_1 = planning_frame_1
    self.eef1_link = eef1_link
    self.group_names = group_names
    self.jointNames = [
        'panda_1_joint1',
        'panda_1_joint2',
        'panda_1_joint3',
        'panda_1_joint4',
        'panda_1_joint5',
        'panda_1_joint6',
        'panda_1_joint7',
        'panda_1_finger_joint1',
    ]
    rospy.loginfo('endInit')


  def open_hand_1(self, wait=True):
    joint_goal = self.hand_1.get_current_joint_values()
    joint_goal[0] = 0.0399

    self.hand_1.set_goal_joint_tolerance(0.001)
    self.hand_1.go(joint_goal, wait=wait)

    self.hand_1.stop()

    current_joints = self.hand_1.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def close_hand_1(self, wait=True):
    joint_goal = self.hand_1.get_current_joint_values()
    joint_goal[0] = 0.0001

    self.hand_1.set_goal_joint_tolerance(0.001)
    self.hand_1.go(joint_goal, wait=wait)

    if (not wait ):
      return

    self.hand_1.stop()

    current_joints = self.hand_1.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)



  def go_to_joint_state_1(self, wait=True, pos=[0, -pi/4, 0, -pi/2, 0, pi/3, 0]):
    joint_goal = self.move_group_1.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -pi/4
    joint_goal[2] = 0
    joint_goal[3] = -pi/2
    joint_goal[4] = 0
    joint_goal[5] = pi/3
    joint_goal[6] = 0

    self.move_group_1.go(joint_goal, wait=wait)

    self.move_group_1.stop()

    current_joints = self.move_group_1.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def go_to_joint_state_2(self, wait=True, pos=[0, -pi/4, 0, -pi/2, 0, pi/3, 0]):
    joint_goal = self.move_group_2.get_current_joint_values()
    joint_goal[0] = pos[0]
    joint_goal[1] = pos[1]
    joint_goal[2] = pos[2]
    joint_goal[3] = pos[3]
    joint_goal[4] = pos[4]
    joint_goal[5] = pos[5]
    joint_goal[6] = pos[6]

    self.move_group_2.go(joint_goal, wait=wait)

    self.move_group_2.stop()

    current_joints = self.move_group_2.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def go_to_joint_state_both(self, wait=True, pos1=[0, -pi/4, 0, -pi/2, 0, pi/3, 0], pos2=[0, -pi/4, 0, -pi/2, 0, pi/3, 0]):
    joint_goal = self.move_group_1.get_current_joint_values()
    joint_goal[0]  = pos1[0]
    joint_goal[1]  = pos1[1]
    joint_goal[2]  = pos1[2]
    joint_goal[3]  = pos1[3]
    joint_goal[4]  = pos1[4]
    joint_goal[5]  = pos1[5]
    joint_goal[6]  = pos1[6]

    self.move_group_1.go(joint_goal, wait=wait)

    self.move_group_1.stop()

    current_joints = self.move_group_1.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def go_to_pose_goal_1(self, pos, ori):
    pose_goal = geometry_msgs.msg.Pose()
    q = quaternion_from_euler(ori[0], ori[1], ori[2])
    pose_goal.orientation = geometry_msgs.msg.Quaternion(*q)
    pose_goal.position.x = pos[0]
    pose_goal.position.y = pos[1]
    pose_goal.position.z = pos[2]

    self.move_group_1.set_goal_orientation_tolerance(0.0001)
    self.move_group_1.set_goal_position_tolerance(0.0001)

    self.move_group_1.set_pose_target(pose_goal)

    plan = self.move_group_1.go(wait=True)
    self.move_group_1.stop()
    self.move_group_1.clear_pose_targets()

    current_pose = self.move_group_1.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)


  def go_to_pose_goal_both(self, pos1, pos2, ori1, ori2):
    pose_goal1 = geometry_msgs.msg.Pose()
    q = quaternion_from_euler(ori1[0], ori1[1], ori1[2])
    pose_goal1.orientation = geometry_msgs.msg.Quaternion(*q)
    pose_goal1.position.x = pos1[0]
    pose_goal1.position.y = pos1[1]
    pose_goal1.position.z = pos1[2]

    self.move_group_1.set_goal_orientation_tolerance(0.0001)
    self.move_group_1.set_goal_position_tolerance(0.0001)

    self.move_group_1.set_pose_target(pose_goal1, end_effector_link=self.eef1_link)

    plan = self.move_group_1.go(wait=True)
    self.move_group_1.stop()
    self.move_group_1.clear_pose_targets()

    current_pose1 = self.move_group_1.get_current_pose(self.eef1_link).pose
    return all_close(pose_goal1, current_pose1, 0.01)


  def plan_cartesian_path_1(self, scale=1):
    waypoints = []

    wpose = self.move_group_1.get_current_pose().pose
    wpose.position.z -= scale * 0.1  # First move up (z)
    wpose.position.y += scale * 0.2  # and sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= scale * 0.1  # Third move sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = self.move_group_1.compute_cartesian_path(
                                         waypoints,   # waypoints to follow
                                         0.01,        # eef_step
                                         0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction


  def plan_cartesian_path_2(self, scale=1):
    waypoints = []

    wpose = self.move_group_2.get_current_pose().pose
    wpose.position.z -= scale * 0.1  # First move up (z)
    wpose.position.y += scale * 0.2  # and sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= scale * 0.1  # Third move sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = self.move_group_2.compute_cartesian_path(
                                         waypoints,   # waypoints to follow
                                         0.01,        # eef_step
                                         0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction


  def plan_cartesian_path_both(self, scale=1):
    waypoints = []

    wpose = self.move_group_1.get_current_pose(self.eef1_link).pose
    wpose.position.z -= scale * 0.1  # First move up (z)
    wpose.position.y += scale * 0.2  # and sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= scale * 0.1  # Third move sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = self.move_group_1.compute_cartesian_path(
                                         waypoints,   # waypoints to follow
                                         0.01,        # eef_step
                                         0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction



  def display_trajectory(self, plan):
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory)


  def execute_plan_1(self, plan):
    self.move_group_1.execute(plan, wait=True)


  def execute_plan_2(self, plan):
    self.move_group_2.execute(plan, wait=True)


  def execute_plan_both(self, plan):
    self.move_group_1.execute(plan, wait=True)

  def callback(self, msg, arm):
    joint_goal = self.move_group_1.get_current_joint_values()
    joint_goal[self.jointNames.index(arm)] = msg.position[0]
    self.move_group_1.go(joint_goal, wait=False)

  def setupSub(self):
    for i in self.jointNames:
      rospy.Subscriber(i, JointState, self.callback, i)
      rospy.loginfo('setting up joint {}'.format(i))  

def close_move_1(robot):

    robot.close_hand_1()

def close_move_2(robot):

    robot.close_hand_2()
    robot.go_to_pose_goal_1([-0.6, -0.1, 1.42], [-pi/2, -pi/4, 0])
    

def main():
  try:
    robot = DoublePandaControl()
    rospy.loginfo("init start")
    # Init
    #robot.go_to_joint_state_both()
    #robot.open_hand_1()

    # One armed pick and place
    #robot.go_to_pose_goal_1([-0.3, 0.7, 1.15], [pi, 0, -pi/4])
    #robot.go_to_pose_goal_1([-0.2, 0.7, 1.13], [pi, 0, -pi/4])
    #robot.go_to_pose_goal_1([-0.25, 0.75, 1.4], [pi, 0, -pi/4])
    #robot.go_to_pose_goal_1([-0.3, 1.0, 1.2], [pi, 0, 0])

    # Two armed pick and move
    #robot.go_to_pose_goal_both([0.15, 0.7, 1.15], [-0.15, 0.7, 1.29], [-pi/2, -pi/4, pi/2], [pi/2, pi/4, pi/2])
    #robot.go_to_pose_goal_both([0.1, 0.7, 1.15], [-0.1, 0.7, 1.25], [-pi/2, -pi/4, pi/2], [pi/2, pi/4, pi/2])
    #robot.close_hand_1()

    #robot.go_to_pose_goal_both([0.1, 0.7, 1.35], [-0.1, 0.7, 1.45], [-pi/2, -pi/4, pi/2], [pi/2, pi/4, pi/2])
    #robot.go_to_pose_goal_both([0.1, 0.8, 1.35], [-0.1, 0.8, 1.45], [-pi/2, -pi/4, pi/2], [pi/2, pi/4, pi/2])
    #robot.open_hand_1()
    robot.setupSub()
    rospy.spin()

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

#!/usr/bin/env python3

# Copyright 1996-2020 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""universal_robot_ros controller."""

import argparse
import rospy

#Don't be controlling the robot from SUPERVISOR!!!!!!
from controller import Robot, Supervisor
from joint_state_publisher import JointStatePublisher
from trajectory_follower import TrajectoryFollower
from gripper_command import GripperCommander
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Pose, Point

parser = argparse.ArgumentParser()
parser.add_argument('--node-name', dest='nodeName', default='double_panda', help='Specifies the name of the node.')
arguments, unknown = parser.parse_known_args()

rospy.init_node(arguments.nodeName, disable_signals=True)

jointPrefix = rospy.get_param('prefix', '')
if jointPrefix:
    print('Setting prefix to %s' % jointPrefix)

robot = Supervisor()
nodeName = arguments.nodeName + '/' if arguments.nodeName != 'ur_driver' else ''
jointStatePublisher = JointStatePublisher(robot, jointPrefix, nodeName)
gripperCommander1 = GripperCommander(robot, jointStatePublisher, jointPrefix, 'panda_1')
trajectoryFollower1 = TrajectoryFollower(robot, jointStatePublisher, jointPrefix, 'panda_1')
trajectoryFollower1.start()
gripperCommander1.start()
init_pos = {
    "panda_1_joint1": 0.000,
    "panda_1_joint2": -0.785,
    "panda_1_joint3": 0.0,
    "panda_1_joint4": -2.356,
    "panda_1_joint5": 0.0,
    "panda_1_joint6": 1.57,
    "panda_1_joint7": 0.4,
            }
for jt in init_pos:
    robot.getMotor(jt).setPosition(init_pos[jt])

eePub = rospy.Publisher('panda_1_ee', Pose, queue_size=1)
# we want to use simulation time for ROS
clockPublisher = rospy.Publisher('clock', Clock, queue_size=1)
if not rospy.get_param('use_sim_time', False):
    rospy.logwarn('use_sim_time is not set!')

timestep = int(robot.getBasicTimeStep())
while robot.step(timestep) != -1 and not rospy.is_shutdown():
    jointStatePublisher.publish()
    trajectoryFollower1.update()
    gripperCommander1.update()
    # pulish simulation clock
    ptemp = Pose()
    ptemp.position.x = robot.getFromDef('PANDA_1_HAND').getPosition()[0]
    ptemp.position.y = robot.getFromDef('PANDA_1_HAND').getPosition()[1]
    ptemp.position.z = robot.getFromDef('PANDA_1_HAND').getPosition()[2]
    ptemp.orientation.x = robot.getFromDef('PANDA_1_HAND').getOrientation()[0]
    ptemp.orientation.y = robot.getFromDef('PANDA_1_HAND').getOrientation()[1]
    ptemp.orientation.z = robot.getFromDef('PANDA_1_HAND').getOrientation()[2]
    ptemp.orientation.w = robot.getFromDef('PANDA_1_HAND').getOrientation()[3]
    eemsg = ptemp
    msg = Clock()
    time = robot.getTime()
    msg.clock.secs = int(time)
    # round prevents precision issues that can cause problems with ROS timers
    msg.clock.nsecs = round(1000 * (time - msg.clock.secs)) * 1.0e+6
    clockPublisher.publish(msg)
    eePub.publish(eemsg)

import json
from sensor_msgs.msg import JointState
import rospy
import moveit_commander


def loadJson(filename):
    with open(filename) as f:
        data = json.load(f)
    return data


class PublishJangles:
    def __init__(self):
        self.publishers = {}
        # self.publish_1 = rospy.Publisher("/panda_1_joint1", JointState, queue_size=1)
        # self.publish_2 = rospy.Publisher("/panda_1_joint2", JointState, queue_size=1)
        # self.publish_3 = rospy.Publisher("/panda_1_joint3", JointState, queue_size=1)
        # self.publish_4 = rospy.Publisher("/panda_1_joint4", JointState, queue_size=1)
        # self.publish_5 = rospy.Publisher("/panda_1_joint5", JointState, queue_size=1)
        # self.publish_6 = rospy.Publisher("/panda_1_joint6", JointState, queue_size=1)
        # self.publish_7 = rospy.Publisher("/panda_1_joint7", JointState, queue_size=1)
        self.jointInfo = {}
        self.noneInfo = {}
        self.subArr = []
        self.state = "UNLIMITED"
        group_name = "panda_1_arm"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

    def setup_sub_pub_pairs(self, pub_name):
        if pub_name not in self.publishers.keys():
            self.publishers[pub_name] = rospy.Publisher(pub_name, JointState, queue_size=1)

    def setup_new_limits(self, data):
        rospy.loginfo('Setting up new limits')
        rospy.loginfo(data)
        joint_names = data.keys()
        for i in joint_names:
            i = data[i]
            # sub_name = i['sub']
            pub_name = i['pub']
            joint_limit_angle_min = i['Amin']
            joint_limit_angle_max = i['Amax']
            joint_limit_torque_min = i['Tmin']
            joint_limit_torque_max = i['Tmax']
            failure_type = i['ftype']
            self.jointInfo[pub_name] = {'Amin': joint_limit_angle_min, 'Amax': joint_limit_angle_max,
                                        'Tmin': joint_limit_torque_min, 'Tmax': joint_limit_torque_max,
                                        'FailureClass': failure_type}
            self.setup_sub_pub_pairs(pub_name)

    def set_none_limits(self, filename):
        with open(filename) as f:
            data = json.load(f)
        rospy.loginfo('Setting up new limits')
        rospy.loginfo(data)
        joint_names = data.keys()
        for i in joint_names:
            i = data[i]
            # sub_name = i['sub']
            pub_name = i['pub']
            joint_limit_angle_min = i['Amin']
            joint_limit_angle_max = i['Amax']
            joint_limit_torque_min = i['Tmin']
            joint_limit_torque_max = i['Tmax']
            failure_type = i['ftype']
            self.noneInfo[pub_name] = {'Amin': joint_limit_angle_min, 'Amax': joint_limit_angle_max,
                                       'Tmin': joint_limit_torque_min, 'Tmax': joint_limit_torque_max,
                                       'FailureClass': failure_type}

    def publish_ujoint_angles(self):

        state_1 = [1.2, -0.5, 1.5, -1., 0.789, 2., 0.75]
        state_2 = [-1.2, .5, -.3, -2., -1.2, 1.5, -.75]

        while True:
            joint_goal = self.move_group.get_current_joint_values()
            for idx, joint_value in enumerate(joint_goal):
                joint_goal[idx] = state_1[idx]
            rospy.loginfo("Moving to State 1")
            self.move_group.go(joint_goal, wait=True)
            self.move_group.stop()
            res = input("press 'ENTER' to Continue, 'l' to limit\n")
            if res == 'l':
                self.state = "LIMITED"
                self.publish_ljoint_angles()
                break
            joint_goal = self.move_group.get_current_joint_values()
            for idx, joint_value in enumerate(joint_goal):
                joint_goal[idx] = state_2[idx]
            rospy.loginfo("Moving to State 2")
            self.move_group.go(joint_goal, wait=True)
            self.move_group.stop()
            res = input("press 'ENTER' to Continue, 'l' to limit\n")
            if res == 'l':
                self.state = "LIMITED"
                self.publish_ljoint_angles()
                break

    def publish_ljoint_angles(self):
        state_1 = [1.2, -0.5, 1.5, -1., 0.789, 2., 0.75]
        state_2 = [-1.2, .5, -.3, -2., -1.2, 1.5, -.75]
        self.setup_new_limits(loadJson('fixedJoint.json'))
        rospy.loginfo("Joints Limited")
        rospy.loginfo("New Joint Limits:")
        val_log = []
        for vals in self.jointInfo.values():
            rospy.loginfo(vals)
            val_log.append(vals)

        while True:
            joint_goal = self.move_group.get_current_joint_values()
            # angle = 0
            for idx, joint_value in enumerate(joint_goal):
                if val_log[idx]["FailureClass"] == "angle":
                    if state_1[idx] < val_log[idx]["Amin"]:
                        angle = val_log[idx]["Amin"]
                    elif state_1[idx] > val_log[idx]["Amax"]:
                        angle = val_log[idx]["Amax"]
                    else:
                        angle = state_1[idx]
                elif val_log[idx]["FailureClass"] == "fixed":
                    angle = val_log[idx]["Amax"]
                elif val_log[idx]["FailureClass"] == "None":
                    angle = state_1[idx]
                else:
                    rospy.logerr("No Failure Class")
                    angle = state_1[idx]
                joint_goal[idx] = angle
            rospy.loginfo("Moving to State 1")
            self.move_group.go(joint_goal, wait=True)
            self.move_group.stop()
            res = input("press 'ENTER' to Continue, 'u' to delimit\n")
            if res == 'u':
                self.state = "UNLIMITED"
                self.publish_ujoint_angles()
                break
            joint_goal = self.move_group.get_current_joint_values()
            # angle = 0
            for idx, joint_value in enumerate(joint_goal):
                if val_log[idx]["FailureClass"] == "angle":
                    if state_2[idx] < val_log[idx]["Amin"]:
                        angle = val_log[idx]["Amin"]
                    elif state_2[idx] > val_log[idx]["Amax"]:
                        angle = val_log[idx]["Amax"]
                    else:
                        angle = state_2[idx]
                elif val_log[idx]["FailureClass"] == "fixed":
                    angle = val_log[idx]["Amax"]
                elif val_log[idx]["FailureClass"] == "None":
                    angle = state_2[idx]
                else:
                    rospy.logerr("No Failure Class")
                    angle = state_2[idx]
                joint_goal[idx] = angle
            rospy.loginfo("Moving to State 2")
            self.move_group.go(joint_goal, wait=True)
            self.move_group.stop()
            res = input("press 'ENTER' to Continue, 'u' to delimit\n")
            if res == 'u':
                self.state = "LIMITED"
                self.publish_ujoint_angles()
                break


if __name__ == "__main__":
    rospy.init_node('joint_publisher')
    rospy.loginfo("initing the joint publishing class")
    pj = PublishJangles()
    if pj.state == "UNLIMITED":
        pj.publish_ujoint_angles()
    elif pj.state == "LIMITED":
        pj.publish_ljoint_angles()

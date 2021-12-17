import rospy
import json
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class LimitingJointClass:
    def __init__(self):
        self.jointInfo = {}
        self.noneInfo = {}
        self.publishers = {}
        self.subArr = []

    # This is the only function you want to call from the outside
    # You can call this to set new limits at any time
    def setupJointLimits(self, filename):
        rospy.logdebug('loading new filename {}'.format(filename))
        self.setup_new_limits(self.loadJson(filename))

    def loadJson(self, filename):
        with open(filename) as f:
            data = json.load(f)
        return data

    def setup_sub_pub_pairs(self, sub_name, pub_name):
        if not pub_name in self.publishers.keys():
            self.publishers[pub_name] = rospy.Publisher(pub_name, JointState, queue_size=1)
            self.subArr.append(rospy.Subscriber(sub_name, JointState, self.callback, (pub_name)))

    def setup_new_limits(self, data):
        rospy.loginfo('Setting up new limits')
        rospy.loginfo(data)
        joint_names = data.keys()
        for i in joint_names:
            i = data[i]
            sub_name = i['sub']
            pub_name = i['pub']
            joint_limit_angle_min = i['Amin']
            joint_limit_angle_max = i['Amax']
            joint_limit_torque_min = i['Tmin']
            joint_limit_torque_max = i['Tmax']
            failure_type = i['ftype']
            self.jointInfo[pub_name] = {'Amin': joint_limit_angle_min, 'Amax': joint_limit_angle_max,
                                        'Tmin': joint_limit_torque_min, 'Tmax': joint_limit_torque_max,
                                        'FailureClass': failure_type}
            self.setup_sub_pub_pairs(sub_name, pub_name)

    def set_none_limits(self, filename):
        with open(filename) as f:
            data = json.load(f)
        rospy.loginfo('Setting up new limits')
        rospy.loginfo(data)
        joint_names = data.keys()
        for i in joint_names:
            i = data[i]
            sub_name = i['sub']
            pub_name = i['pub']
            joint_limit_angle_min = i['Amin']
            joint_limit_angle_max = i['Amax']
            joint_limit_torque_min = i['Tmin']
            joint_limit_torque_max = i['Tmax']
            failure_type = i['ftype']
            self.noneInfo[pub_name] = {'Amin': joint_limit_angle_min, 'Amax': joint_limit_angle_max,
                                       'Tmin': joint_limit_torque_min, 'Tmax': joint_limit_torque_max,
                                       'FailureClass': failure_type}

    def callback(self, msg, pub_name):
        jointDict = self.jointInfo[pub_name]
        noneDict = self.noneInfo[pub_name]
        # This can be changed to fit failure types. Just dont know what to do here tbh.
        failure_type = jointDict['FailureClass']
        rospy.logdebug('Initial msg')
        rospy.loginfo(pub_name)
        rospy.loginfo(msg)
        rospy.loginfo(Float64(msg.position[0]))
        if failure_type == 'angle' or failure_type == 'both':
            msg.position = (float(np.clip(msg.position[0], jointDict['Amin'], jointDict['Amax'])),)
        elif failure_type == 'fixed':
            msg.position = (float(np.clip(msg.position[0], jointDict['Amin'], jointDict['Amax'])),)
        elif failure_type == 'None':
            msg.position = (float(np.clip(msg.position[0], noneDict['Amin'], noneDict['Amax'])),)
        # if failure_type == 'torque' or failure_type == 'both':
        #    msg.torque = np.clip(msg.angle, jointDict['Tmin'], jointDict['Tmax'])
        rospy.logdebug('Limited msg')
        rospy.logdebug(pub_name)
        rospy.logdebug(msg)
        self.publishers[pub_name].publish(msg)


if __name__ == "__main__":
    rospy.init_node('robot_limiter', anonymous=True)
    rospy.loginfo("initing the joint limiting class")
    test = LimitingJointClass()
    test.setupJointLimits('testJointLimits.json')
    # test.setupJointLimits('fixedJoint.json')
    test.set_none_limits('unlimited_joints.json')
    rospy.loginfo("done")
    # You can use a rosrate here or something to wait to set new limits.
    rospy.spin()

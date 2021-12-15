import rospy
import json
import numpy as np
from webots_ros.msgs import JointCommand


class LimitingJointClass:
    def __init__(self):
        self.jointInfo = {}
        self.publishers = {}
        self.subArr = []

    #This is the only function you want to call from the outside
    #You can call this to set new limits at any time
    def setupJointLimits(self, filename):
        rospy.logdebug('loading new filename', filename)
        self.setup_new_limits(self.loadJson(filename))

    def loadJson(self, filename):
        with open(filename) as f:
            data = json.load(f)
        return data

    def setup_sub_pub_pairs(self, sub_name, pub_name):
        if pub_name in self.publishers.keys():
            self.publishers[pub_name] = rospy.Publisher(pub_name, JointCommand, queue_size=1)
            self.subArr.append(rospy.Subscriber(sub_name, JointCommand, self.callback, (pub_name)))

    def setup_new_limits(self, data):
        rospy.logdebug('Setting up new limits')
        rospy.logdebug(data)
        joint_names = data.keys()
        for i in joint_names:
            sub_name = i['sub']
            pub_name = i['pub']
            joint_limit_angle_min = i['Amin']
            joint_limit_angle_max = i['Amax']
            joint_limit_torque_min = i['Tmin']
            joint_limit_torque_max = i['Tmax']
            failure_type = i['ftype']
            self.jointInfo[pub_name] = {'Amin':joint_limit_angle_min, 'Amax':joint_limit_angle_max,
                                        'Tmin':joint_limit_torque_min, 'Tmax':joint_limit_torque_max,
                                        'FailureClass':failure_type}
            self.setup_sub_pub_pairs(sub_name, pub_name)

    def callback(self, msg, pub_name):
        jointDict = self.jointInfo[pub_name]
        #This can be changed to fit failure types. Just dont know what to do here tbh.
        failure_type = jointDict['FailureClass']
        rospy.logdebug('Initial msg', pub_name, msg)
        if failure_type == 'angle' or failure_type == 'both':
            msg.angle = np.clip(msg.angle, jointDict['Amin'], jointDict['Amax'])
        if failure_type == 'angle' or failure_type == 'both':
            msg.torque = np.clip(msg.angle, jointDict['Tmin'], jointDict['Tmax'])
        rospy.logdebug('Limited msg', pub_name, msg)
        self.publishers[pub_name].publish(msg)


if __name == "__main__":
    rospy.info("initing the joint limiting class")
    test = LimitingJointClass()
    test.setupJointLimits('testJointLimits.json')
    #You can use a rosrate here or something to wait to set new limits.
    rospy.spin()

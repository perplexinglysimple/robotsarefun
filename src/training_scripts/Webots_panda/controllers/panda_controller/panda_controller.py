from math import pi
from controller import Supervisor
import numpy as np
import math as m
from pytransform3d import rotations as pr
from scipy.spatial.transform import Rotation as R


###########################################################################################
## Some class definitions -- same as previous HW
###########################################################################################
class DH:
    a = 0.0
    d = 0.0
    alpha = 0.0
    theta = 0.0

    def __init__(self, a=0.0, d=0.0, alpha=0.0, theta=0.0):
        self.a = a
        self.d = d
        self.alpha = alpha
        self.theta = theta

    def print(self):
        print("a = {} d = {} alpha = {} theta = {} \n".format(self.a, self.d, self.alpha, self.theta))

    def dhMatrix(self):
        """Returns a numpy-based Denavit-Hartenberg matrix"""
        # M = np.matrix([[m.cos(self.theta), -m.sin(self.theta) * m.cos(self.alpha),
        #                 m.sin(self.theta) * m.sin(self.alpha), self.a * m.cos(self.theta)],
        #                [m.sin(self.theta), m.cos(self.theta) * m.cos(self.alpha),
        #                 -m.cos(self.theta) * m.sin(self.alpha), self.a * m.sin(self.theta)],
        #                [0, m.sin(self.alpha), m.cos(self.alpha), self.d],
        #                [0, 0, 0, 1]])

        M = np.matrix([[m.cos(self.theta), -m.sin(self.theta), 0, self.a],
                       [m.sin(self.theta)*m.cos(self.alpha), m.cos(self.theta)*m.cos(self.alpha), -m.sin(self.alpha), -self.d*m.sin(self.alpha)],
                       [m.sin(self.theta)*m.sin(self.alpha), m.cos(self.theta)*m.sin(self.alpha), m.cos(self.alpha), self.d*m.cos(self.alpha)],
                       [0, 0, 0, 1]])

        return M

    def setTheta(self, theta):
        self.theta = theta

    def getTheta(self):
        return self.theta


class Joint:
    """A joint has a DH object, a name and a sensor"""
    dh = DH()

    joint_name = ""
    joint_node = None

    sensor_name = ""
    sensor_node = None

    def __init__(self, dh=DH(), joint_name="", sensor_name=""):
        self.dh = dh
        self.joint_name = joint_name
        joint_node = supervisor.getDevice(joint_name)

        if joint_node is None:
            print('FAILED to find joint with name: {}'.format(joint_name))
        else:
            print('Found motor: {}'.format(joint_name))
            self.joint_node = joint_node

        self.sensor_name = sensor_name
        sensor_node = supervisor.getDevice(sensor_name)

        if sensor_node is None:
            print('FAILED to find sensor with name: {}'.format(sensor_name))
        else:
            self.sensor_node = sensor_node
            self.sensor_node.enable(SENSOR_SAMPLE_RATE)
            print('Found sensor: {}'.format(sensor_name))

    def setTheta(self, theta):
        self.dh.setTheta(theta)

    def getTheta(self):
        return self.dh.getTheta()

    def printDH(self):
        self.dh.print()

    def print(self):
        print("joint_name = {} sensor_name = {} \n".format(self.joint_name, self.sensor_name))
        self.printDH()


class Chain:
    """A chain is a sequence of joints, typically referred to as q"""
    q = []
    dof = 0

    def __init__(self, supervisor, q=[]):
        self.supervisor = supervisor

        self.q = q
        self.dof = len(self.q)

    def getTheta(self):
        angles = []
        for joint in self.q:
            angles.append(joint.getTheta())
        return angles

    def printDH(self):
        for i, q_i in enumerate(self.q):
            q_i.printDH()

    def print(self):
        for i, q_i in enumerate(self.q):
            q_i.print()

    def printSensorValues(self):
        sensor_values = []
        for i in range(self.dof):
            sensor_node = self.q[i].sensor_node
            sensor_values.append(sensor_node.getValue())
        print('sensor values: ', sensor_values)

    def printjoints(self):
        for i, q_i in enumerate(self.q):
            print("joint {} position: {}".format(i, q_i.getTheta()))

    def moveJoints(self, q=[]):
        if (self.dof != len(q)):
            print("moveJoints: invalid input")
            print("self.dof: {}, len(q): {}".format(self.dof, len(q)))
            return
        for i in range(self.dof):
            # Setting the theta in the kinematics
            self.q[i].setTheta(q[i])
            # Setting the theta in the simulator
            joint_node = self.q[i].joint_node
            joint_node.setPosition(q[i])

    def moveInJointIncrements(self, start_joints, goal_joints, time_step=10, num_pts=1000):
        joint_traj = []
        for i in range(self.dof):
            j = np.linspace(start_joints[i], goal_joints[i], num_steps, dtype=float)
            joint_traj.append(j)

        # print('joint trajectory: ', joint_traj)
        step_num = 0
        while supervisor.step(TIME_STEP) != -1 and step_num < num_steps:
            j = []
            for i in range(self.dof):
                j_traj = joint_traj[i]
                j_step = j_traj[step_num]
                j.append(j_step)
            self.moveJoints(j)
            self.moveTransforms()
            print('current joint states: ', self.printjoints())
            step_num += 1

    def forwardKinematics(self, base=np.identity(4), joint_num=-1):
        """Computes the end-effector pose from the robot's DH parameters and joint configuration"""
        #
        # pEE = base                # Cumulative pose of the End Effector
        #                           # (initially set up as the base of the robot)

        pEE = base  # Cumulative pose of the End Effector
        # (initially set up as the base of the robot)
        if joint_num == -1:
            for joint in self.q:
                pEE = np.matmul(pEE, joint.dh.dhMatrix())  # TODO: shit broke
        else:
            for i in range(joint_num):
                pEE = np.matmul(pEE, self.q[i].dh.dhMatrix())

        return pEE

        # return pEE

    def computeJacobian(self):
        """This function computes the jacobian of the chain.
        Its ouput is a 6xn jacobian matrix"""
        Js = []
        o_n = (self.forwardKinematics()[:3, 3])
        for dof in range(0, self.dof):
            o_i = self.forwardKinematics(joint_num=dof)[:3, 3]
            z_i = self.forwardKinematics(joint_num=dof)[:3, 2]
            if dof is 0:
                o_i = o_i.reshape((3, 1))
                z_i = z_i.reshape((3, 1))
            first = np.cross(z_i.T, (o_n-o_i).T)
            J_i = np.squeeze(np.array(np.concatenate((first, z_i.T), axis=1).T))
            Js.append(J_i)

        jac = np.matrix(Js).T
        return jac

    def checkJacobian(self):
        q1 = np.array([joint.getTheta() for joint in self.q])
        q1_fk = self.forwardKinematics()
        q1_xyz = np.squeeze(np.array([q1_fk[:3, 3][0], q1_fk[:3, 3][1], q1_fk[:3, 3][2]]))
        q1_rot = R.from_matrix(q1_fk[0:3, 0:3])
        q1_rot = q1_rot.as_euler('xyz')
        q1_fk = np.concatenate((q1_xyz, q1_rot))
        delta_q = np.array([0.01, -0.01, 0.01, -0.01, 0.01, 0.01, -0.01])
        q2 = q1+delta_q
        for num, angle in enumerate(q2):
            self.q[num].setTheta(angle)
        q2_fk = self.forwardKinematics()
        q2_xyz = np.squeeze(np.array([q2_fk[:3, 3][0], q2_fk[:3, 3][1], q2_fk[:3, 3][2]]))
        q2_rot = R.from_matrix(q2_fk[0:3, 0:3])
        q2_rot = q2_rot.as_euler('xyz')
        q2_fk = np.concatenate((q2_xyz, q2_rot))
        lhs = q1_fk + np.matmul(self.computeJacobian(), delta_q)
        diff = lhs - q2_fk

        return diff

    def gd_joint_angles(self, ee_pos_d):
        """This function computes the inverse kinematics using gradient descent.
        It receives a desired end-effector position (as a 3x1 vector)
        It returns the solution of the IK task (as a nx1 vector)


        +20 extra credit if you make it work for rotations too!!"""

        # sol = zeros((self.dof, 1))
        it = 0
        # joint_angles = self.q
        alpha = .1
        converge = np.inf
        q = np.transpose(np.matrix([joint.getTheta() for joint in self.q]))
        while it < 10000 and converge > 0.0001:

            fk = self.forwardKinematics()
            fk_xyz = np.squeeze(np.array([fk[:3, 3][0], fk[:3, 3][1], fk[:3, 3][2]]))
            fk_rot = R.from_matrix(fk[0:3, 0:3])
            fk_rot = fk_rot.as_euler('xyz')
            fk = np.concatenate((fk_xyz, fk_rot))
            diff = alpha*np.transpose(self.computeJacobian())*np.transpose(np.matrix(ee_pos_d-fk))
            # print("CHECK THESE")
            # print(np.shape(np.subtract(np.transpose(np.matrix(q)), diff)))
            # raise Exception
            q = np.subtract(q, diff)
            # print(np.shape(q))
            # raise Exception

            converge = np.linalg.norm(ee_pos_d - fk)
            it += 1
        sol = q

        # for jangle, idx in enumerate(sol):
        #     if jangle > 2*np.pi:
        #         sol[idx] = jangle % 2*np.pi

        # Fill this up

        return sol


###########################################################################################
## Initializations
###########################################################################################

# Create the Robot instance.
# robot = Robot()
# In this homework we will use the supervisor, therefore we create a supervisor object
# which inherits from a Robot objects so we can call all the same functions
supervisor = Supervisor()

TIME_STEP = 10
SENSOR_SAMPLE_RATE = 1

# DH is your array of DH elements to pass to the constructor of arm
# a, d, alpha, theta
dh = [DH(0.0, 0.333, 0.0, 0.0),
      DH(0.0, 0.0, -np.pi / 2.0, 0.0),
      DH(0.0, 0.316, np.pi / 2.0, 0.0),
      DH(0.0825, 0.0, np.pi / 2.0, 0.0),
      DH(-0.0825, 0.384, -np.pi / 2.0, 0.0),
      DH(0.0, 0.0, np.pi / 2.0, 0.0),
      DH(0.088, 0.0, np.pi / 2.0, 0.0)]

# These are the arrays of joint names and sensor names to control the robot and retrieve joint angles
joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6',
               'panda_joint7']
sensor_names = ['panda_joint1_sensor', 'panda_joint2_sensor', 'panda_joint3_sensor', 'panda_joint4_sensor',
                'panda_joint5_sensor', 'panda_joint6_sensor', 'panda_joint7_sensor']

joints = []
for i in range(len(dh)):
    joints.append(Joint(dh[i], joint_names[i], sensor_names[i]))

# Creation of a chain object as composition of multiple joints
arm = Chain(supervisor, joints)

supervisor.step(TIME_STEP)
# Printing parameters just to check that everything works
print("*********************************************")
# arm.print()
# arm.printjoints()
# arm.printSensorValues()
print(arm.forwardKinematics())

# Let's move the joints to a new position
arm.moveJoints([0.2400003-np.pi, 0.2400003, 0, -.5, 2.9671-np.pi, 2.3, np.pi/4])
# fk = arm.forwardKinematics()
# print("THIS DH IS IMPORTANT")
# print(np.squeeze(np.array([fk[:3, 3][0], fk[:3, 3][1], fk[:3, 3][2]])))
# rot = R.from_matrix(fk[0:3, 0:3])
# rot = rot.as_rotvec()
# angle = np.linalg.norm(rot)
# axis = np.divide(rot, angle)
# print(axis, angle)
# print(arm.computeJacobian())
#
# print("JACOBIAN CHECK")
# print(arm.checkJacobian())
# arm.moveJoints([0, 0, 0, 0, 0, 0, 0])
# fk = arm.forwardKinematics()
# print("THIS DH IS IMPORTANT")
# print(np.squeeze(np.array([fk[:3, 3][0], fk[:3, 3][1], fk[:3, 3][2]])))
# rot = R.from_matrix(fk[0:3, 0:3])
# rot = rot.as_rotvec()
# angle = np.linalg.norm(rot)
# axis = np.divide(rot, angle)
# print(axis, angle)
# print(arm.computeJacobian())
#
# print("JACOBIAN CHECK")
# print(arm.checkJacobian())
# arm.moveJoints([0.24, 0.24, 0.24, -0.24, 0.24, 0.7, 0.24])



# run one time step of the robot in order to read the joint positions
# Time step is two seconds so that we can see the difference between STEP 1 and STEP 2
supervisor.step(TIME_STEP)

# print("*********************************************")
# arm.printSensorValues()

###########################################################################################
## Inverse Kinematics Part I -- Compute Jacobians
###########################################################################################

# 1.1 Implement the forwardKinematics function. Use the one in the past assignment and
# adapt it to this code (10pts)
# print(arm.forwardKinematics())
# Does it make sense with the robot in sim?
# If not, why? +10pts EXTRA CREDIT + +10pts EXTRA CREDIT IF YOU FIX IT

# 1.2 Fill up Compute Jacobian function in Chain class and print the jacobian (30pts)

# raise Exception

# 1.3 What informatino can you retrieve by looking at these numbers? Do they make sense?
# Please provide an explaination below, with code and printouts if needed
# (otherwise just use comments) +10pts EXTRA CREDIT


###########################################################################################
## Inverse Kinematics Part II -- Gradient Descent
###########################################################################################

# 2.1 Compute the inverse kinematics with gradient descent (50pts)
# It is literally one line of code from the slides (plus maybe another 20 lines of checks)

# 2.2 Compute the IK for each of these following values:
ee_pos_d_1 = np.array([0.31217954, 0.13414825, .92119663, -2.91880302, -0.20014455,  0.40635089])
print("INITIAL STATE")
initial = arm.forwardKinematics()
initial_angles = np.array([joint.getTheta() for joint in arm.q])
initial_xyz = np.squeeze(np.array([initial[:3, 3][0], initial[:3, 3][1], initial[:3, 3][2]]))
initial_rot = R.from_matrix(initial[0:3, 0:3])
initial_rot = initial_rot.as_euler('xyz')
initial = np.concatenate((initial_xyz, initial_rot))
print(initial)
ee_pos_d_1_joints = (np.array([joint.getTheta() for joint in arm.q])) # + np.array([1, -.3, .3, .3, 1, .3, -.3]))
# print(ee_pos_d_1_joints)
for num, angle in enumerate(ee_pos_d_1_joints):
    arm.q[num].setTheta(angle)
print("PERTURBED")
pert = arm.forwardKinematics()
pert_angles = np.array([joint.getTheta() for joint in arm.q])
pert_xyz = np.squeeze(np.array([pert[:3, 3][0], pert[:3, 3][1], pert[:3, 3][2]]))
pert_rot = R.from_matrix(pert[0:3, 0:3])
pert_rot = pert_rot.as_euler('xyz')
pert = np.concatenate((pert_xyz, pert_rot))
print(pert)

# ee_pos_d_2 = [0.4, 0.4, 0.0]
# ee_pos_d_3 = [0.1, 0.1, 0.5]
# q1 = arm.gd_joint_angles(ee_pos_d_1)
# print("GRADIENT DESCENT!!!")
# new_joint = []
# for num, angle in enumerate(q1):
#     arm.q[num].setTheta(angle)
#     # print(float(angle))
#     arm.q[num].joint_node.setPosition(float(angle))
# # arm.moveJoints(new_joint)
# fk = arm.forwardKinematics()
# fk_xyz = np.squeeze(np.array([fk[:3, 3][0], fk[:3, 3][1], fk[:3, 3][2]]))
# fk_rot = R.from_matrix(fk[0:3, 0:3])
# fk_rot = fk_rot.as_euler('xyz')
# fk = np.concatenate((fk_xyz, fk_rot))
# print("COMPARE THESE")
# print(pert, fk)
# # print(np.squeeze(arm.getTheta()))
# raise Exception

# For each of these three cases, move the joints to the desired position,
# and check if the end effector position makes sense IN CODE (6pts)


# Please comment on the above (4 pts)
# arm.moveJoints([0.24, 0.24, 0.24, -0.24, 0.24, 0.7, 0.24])


while supervisor.step(TIME_STEP) != -1:
    # arm.printSensorValues()
    pass

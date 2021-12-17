import rospy
from geometry_msgs.msg import Pose, Point
from sensor_msgs.msg import JointState
import math

prePosition = None
reachedGoal = False
goalStateChanged = False
actualJPose = None
goalJState  = None
dist        = 0
prevDis     = 9999
timeStuck   = 0
prevTime    = 0


def callback(data):
    global prePosition
    global dist
    global stopChanging
    global reachedGoal
    global timeStuck
    global prevDis
    global prevTime
    if prePosition is None:
        prePosition = data.position
        prevTime = rospy.Time.now()
        return
    #Checking if end is moving
    dist = math.hypot(data.position.x - prePosition.x, data.position.y - prePosition.y,
                      data.position.z - prePosition.z)
    prePosition = data.position
    #checking if we have reached the goal
    if actualJPose is None or goalJState is None:
        prevTime = rospy.Time.now()
        return
    if actualJPose.position[0] == goalJPose.position[0]:
        #Reached Goal
        reachedGoal = True
        timeStuck = 0
    else:
        reachGoal = False
        if dist - prevDis == 0:
            #We are stuck
            timeStuck += rospy.Time.now() - prevTime
            if timeStuck > 2:
                print('Joint 1 stuck')
    prevTime = rospy.Time.now()



def goalCallback(msg):
    global goalJState
    global goalStateChanged
    goalState = msg
    goalStateChanged = True

def resCallback(msg):
    global actualJPose
    actualJPose = msg

rospy.init_node('stupid_detector', anonymous=True)
rospy.Subscriber("panda_1_ee", Pose, callback)
#only checks one joint. Need to make it into a class to check all
rospy.Subscriber("/limited_joint1", JointState, goalCallback)
rospy.Subscriber("/panda_1_joint1", JointState, resCallback)

rospy.spin()

from time import sleep
sleep(10)
while True:
    print("is limited!")
    sleep(.1)

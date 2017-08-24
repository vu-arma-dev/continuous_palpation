#!/usr/bin/env python
from collections import deque
import numpy as np
import rospy
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from std_msgs.msg import UInt32
import PyKDL
from tf_conversions import posemath
from tf import transformations
from dvrk import psm
from time import time

def npToKdlFrame(npFrame):
	quat = transformations.quaternion_from_matrix(npFrame)
	rot = PyKDL.Rotation.Quaternion(quat[0],quat[1],quat[2],quat[3])
	pos = PyKDL.Vector(npFrame[0,3],npFrame[1,3],npFrame[2,3])
	kdlFrame = PyKDL.Frame.Identity()
	kdlFrame.M = rot
	kdlFrame.p = pos
	return kdlFrame

def kdlToNpFrame(kdlFrame):
	npFrame = np.empty((4,4));
	v = kdlFrame.M.UnitX()
	npFrame[0,0:3] = [v.x(), v.y(), v.z()]
	v = kdlFrame.M.UnitY()
	npFrame[1,0:3] = [v.x(), v.y(), v.z()]
	v = kdlFrame.M.UnitZ()
	npFrame[2,0:3] = [v.x(), v.y(), v.z()]
	npFrame[0:3,3] = [kdlFrame.p.x(),kdlFrame.p.y(),kdlFrame.p.z()]
	npFrame[0:3,0:3] = npFrame[0:3,0:3] / np.linalg.norm(npFrame[0:3,0:3], axis=-1)[:, np.newaxis]
	npFrame[3,:] = [0, 0, 0, 1]
	return npFrame;

class ContinuousPalpation:
    def __init__(self, psmName, forceTopic, bufferSize = 50):

        rospy.init_node('continuous_palpation', anonymous=True)
        
        self.f_buffer = deque([], bufferSize)
        self.f_current = np.zeros((3))

        self.trajectory = deque([])

        # Set up subscibers
        self.poseSub = rospy.Subscriber(name = 'set_continuous_palpation_goal',
                                        data_class = PoseStamped,
                                        callback = self.poseCB,
                                        queue_size = 1)
        self.trajSub = rospy.Subscriber(name = 'set_continuous_palpation_trajectory',
                                        data_class = PoseArray,
                                        callback = self.poseArrayCB,
                                        queue_size = 1)
        self.forceSub = rospy.Subscriber(name = forceTopic,
                                         data_class = WrenchStamped,
                                         callback = self.forceCB,
                                         queue_size = 1)

        # Set up publishers
        self.trajStatusPub = rospy.Publisher(name = 'trajectory_length',
                                             data_class = UInt32, 
                                             queue_size = 1)

        self.robot = psm(psmName)

        # TODO make these values not hard coded
        self.rate = rospy.Rate(1000) # 1000hz
        self.fBias = np.array((0 0 1)) # Newtons
        self.period = .5 # Seconds
        self.amplitude = .5 # Newtons

        self.run()

    def run(self):
        nextPose = np.empty((3))
        while not rospy.is_shutdown():
            try:
                nextPose = self.trajectory[0]
            except IndexError:
                continue
            currentPose = self.robot.get_current_position()
            xDotMotion = self.resolvedRates(currentPose, nextPose)
            xDotForce = self.forceAdmittanceControl()
            xDot = hybridPosForce(xDotMotion,xDotForce)
            # self.robot.move(desiredPose, interpolate = False)
            self.trajStatusPub.publish(len(self.trajectory))
            self.rate.sleep()

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    def forceCB(self, data):
        self.f_current = np.array((data.wrench.force.x,
                                   data.wrench.force.y,
                                   data.wrench.force.z))
        self.f_buffer.append(f_current)

    def poseCB(self, data):
        self.trajectory.append( posemath.fromMsg(data.pose))
    
    def poseArrayCB(self, data):
        for pose in data.poses:
            self.trajectory.append( posemath.fromMsg(pose))

    def resolvedRates(self, poseCur, poseDes):
        # TODO write resolved rates
        xDotMotion = np.empty((6,1))
        return xDotMotion

    def computeFRef(self):
        # TODO compute fRef
        mag = np.sin(self.period*time() / (2*np.PI)) * self.amplitude
        displacement = (self.fBias / np.linalg.norm(self.fBias)) * mag
        return self.fBias + displacement

    def forceAdmittanceControl(self):
        # TODO implement force admittance control
        fRef = self.computeFRef()
        xDotForce = np.empty((6,1))
        # [xDotForce.p.x, xDotForce.p.x, xDotForce.p.y] = fRef;
        return x_dot_motion

    def hybridPosForce(self, xDotMotion, xDotForce, poseCur):
        # TODO implement hybrid force position control
        xDot = xDotForce + xDotMotion
        return xDot * (1.0/self.rate) + poseCur

if __name__ == '__main__':
    ContinuousPalpation(psmName = 'PSM1', forceTopic = '/atinetft/raw_wrench')

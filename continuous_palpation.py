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
import copy

from IPython import embed

# DEBUG TOOLS
# import ipdb

class ContinuousPalpation:
    def __init__(self, psmName, forceTopic, bufferSize = 50):

        rospy.init_node('continuous_palpation', anonymous=True)
        
        self.fBuffer = deque([], bufferSize)
        self.fCurrent = PyKDL.Vector(0.0,0.0,0.0)

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
        self.forceProfile = \
        {   'period':0.5, # Seconds
            'amplitude': 0.0, # Newtons, default = 0.5
            'fBiasMag': 0.0, # Newtons, biased force magnitude, default=0.7
            'magnitudeMode': 'bias', # 'bias' or 'sine', or 'sine bias'
            'controlDir':'surf normal', # 'default' or 'surf normal'
            'defaultDir':[0.0,0.0,0.0], # default = [0.0,0.0,1.0]
            'admittanceGains':[ 100.0/1000,\
                                100.0/1000,\
                                100.0/1000], # force admittance gains, (meter/sec)/Newton
            'noiseThresh': 0.08 # Newtown, a threshold value to cancel the noise
        }
        self.resolvedRatesConfig = \
        {   'velMin': 2.0/1000,
            'velMax': 20.0/1000,
            'angVelMin': 2.0/180.0*3.14,
            'angVelMax': 15.0/180.0*3.14,
            'tolPos': 0.5/1000, # positional tolerance
            'tolRot': 1.0/180*3.14, # rotational tolerance
            'velRatio': 10.0, # the ratio of max velocity error radius to tolarance radius, this value >1
            'rotRatio': 5.0,
            'dt': 1.0/1000, # this is the time step of the system. 
                            # if rate=1khz, then dt=1.0/1000. However, 
                            # we don't know if the reality will be the same as desired rate
        }
        self.run()

    def run(self):
        nextPose = PyKDL.Frame.Identity()
        while not rospy.is_shutdown():
            # Publish trajectory length at all times
            self.trajStatusPub.publish(len(self.trajectory))

            # Check if there are any trajectories
            try:
                nextPose = self.trajectory[0]
            except IndexError:
                # If no trajectory do nothing
                continue
            
            # get current and desired robot pose (desired is the top of queue)
            currentPose = self.robot.get_desired_position()
            desiredPose = self.trajectory[0]
            # compute the desired twist "x_dot" from motion command
            xDotMotion = self.resolvedRates(currentPose,desiredPose) # xDotMotion is type [PyKDL.Twist]
            # compute the desired twist "x_dot" from force command
            forceCtrlDir = self.updateForceControlDir()
            xDotForce = self.forceAdmittanceControl(forceCtrlDir) # xDotForce is type [PyKDL.Twist]
            xDot = self.hybridPosForce(xDotMotion,xDotForce,forceCtrlDir)

            # apply the desired twist on the currnet pose
            dt = self.resolvedRatesConfig['dt']
            poseToMove = PyKDL.addDelta(poseCur,xDotMotionForce,sysDT)

            # Check whether we have reached our goal
            if  xDotMotion.vel.Norm() <= self.resolvedRatesConfig['tolPos'] \
            and xDotMotion.rot.Norm() <= self.resolvedRatesConfig['tolRot']:
                self.trajectory.popleft()
                print(len(self.trajectory))

            # Move the robot
            self.robot.move(poseToMove, interpolate = False)
            self.rate.sleep()

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    def forceCB(self, data):
        # The received data needs to be in PyKDL.Vector format
        force = data.wrench.force
        self.fCurrent = PyKDL.Vector(force.x,force.y,force.z)
        self.fBuffer.append(self.fCurrent)
    
    def getAverageForce(self):
        # TODO let's figure out a way to do this using PyKDL
        if len(self.fBuffer) == 0:
            return PyKDL.Vector(0.0,0.0,0.0)
        return np.mean(self.fBuffer)
    
    def poseCB(self, data):
        # Check if timestamp makes sense
        time_diff = (rospy.get_rostime() - data.header.stamp).to_sec()
        if time_diff > 10:
            print("Ignoring old message timestamped %2fs ago", time_diff)
            return
        self.trajectory.append(posemath.fromMsg(data.pose))
        print(len(self.trajectory))

    def poseArrayCB(self, data):
        # Check if timestamp makes sense
        time_diff = (rospy.get_rostime() - data.header.stamp).to_sec()
        if time_diff > 10:
            print("Ignoring old message timestamped %.2fs ago" % time_diff)
            return
        # Fill out pose array
        for pose in data.poses:
            self.trajectory.append(posemath.fromMsg(pose))
        print(len(self.trajectory))

    def resolvedRates(self,currentPose,desiredPose):
        # compute pose error (result in kdl.twist format)
        poseError = PyKDL.diff(currentPose,desiredPose)
        posErrNorm = poseError.vel.Norm()
        rotErrNorm = poseError.rot.Norm()
        # compute velocity magnitude based on position error norm
        if posErrNorm>self.resolvedRatesConfig['tolPos']:
            tolPosition = self.resolvedRatesConfig['tolPos']
            lambdaVel = self.resolvedRatesConfig['velRatio']
            if posErrNorm>(lambdaVel*tolPosition):
                velMag = self.resolvedRatesConfig['velMax']
            else:
                velMax = self.resolvedRatesConfig['velMax']
                velMin = self.resolvedRatesConfig['velMin']
                velMag = velMin + (posErrNorm - tolPosition) * \
                (velMax - velMin)/(tolPosition*(lambdaVel-1))
        else:
            velMag = 0.0
        # compute angular velocity based on rotation error norm
        if rotErrNorm>self.resolvedRatesConfig['tolRot']:
            tolRotation = self.resolvedRatesConfig['tolRot']
            lambdaRot = self.resolvedRatesConfig['rotRatio']
            if rotErrNorm>(lambdaRot*tolRotation):
                angVelMag = self.resolvedRatesConfig['angVelMax']
            else:
                angVelMax = self.resolvedRatesConfig['angVelMax']
                angVelMin = self.resolvedRatesConfig['angVelMin']
                angVelMag = angVelMin + (rotErrNorm - tolRotation) * \
                (angVelMax - angVelMin)/(tolRotation*(lambdaRot-1))
        else:
            angVelMag = 0.0
            # The resolved rates is implemented as Nabil Simaan's notes
        # apply both the velocity and angular velocity in the error pose direction
        desiredTwist = PyKDL.Twist()
        poseError.vel.Normalize() # normalize to have the velocity direction
        desiredTwist.vel = poseError.vel*velMag
        poseError.rot.Normalize() # normalize to have the ang vel direction
        desiredTwist.rot = poseError.rot*angVelMag
        return desiredTwist

    def updateForceControlDir(self):
        # this func updates the force control direction based on specs
        # load the desired force control direction according to different mode
        if self.forceProfile['controlDir'] == 'surf normal':
            # under this condition, 
            # need to compute the force control direction based on f_buffer
            forceCtrlDir = self.getAverageForce()
            # Need to check the norm of the force readings, if too small, treat it as noise
            if (forceCtrlDir.Norm()>self.forceProfile['noiseThresh']):
                forceCtrlDir.Normalize()
            else:
                forceCtrlDir = PyKDL.Vector(\
                    self.forceProfile['defaultDir'][0],
                    self.forceProfile['defaultDir'][1],
                    self.forceProfile['defaultDir'][2])                        
        else: 
            # if in default direction case
            forceCtrlDir = PyKDL.Vector(\
                self.forceProfile['defaultDir'][0],
                self.forceProfile['defaultDir'][1],
                self.forceProfile['defaultDir'][2])
        return forceCtrlDir

    def forceAdmittanceControl(self,forceCtrlDir):
        # compute the desired force magnitude  
        sinMag = np.sin(self.forceProfile['period']*time() / (2*np.pi)) * \
                        self.forceProfile['amplitude']
        fRefMag = sinMag + self.forceProfile['fBiasMag']
        # compute desired force
        desiredForce = forceCtrlDir * fRefMag
        # get the current force
        forceError = self.fCurrent - desiredForce
        # apply admittance gain to compute motion
        desiredTwist = PyKDL.Twist()
        desiredTwist.vel = PyKDL.Vector(\
            forceError.x()*self.forceProfile['admittanceGains'][0],
            forceError.y()*self.forceProfile['admittanceGains'][1],
            forceError.z()*self.forceProfile['admittanceGains'][2])
        desiredTwist.rot = PyKDL.Vector(0.0,0.0,0.0)
        return desiredTwist

    def hybridPosForce(self, xDotMotion, xDotForce, forceCtrlDir):
        # TODO implement hybrid force position control
        # project the force command onto the force control direction
        velForce = self.projectDirection(forceCtrlDir,xDotForce.vel)
        xDotForce.vel = velForce
        # project the motion command onto the null space of force control direction
        velMotion = self.projectNullSpace(forceCtrlDir,xDotMotion.vel)
        xDotMotion.vel = velMotion
        # combine twist 
        xDot = xDotMotion + xDotForce
        return xDot

    @staticmethod
    def projectDirection(projDir,vector):
        # this static method compute the projection of a vector onto a direction
        # both variables need to be in format of PyKDL.Vector
        # compute the projection magnitude of vector onto the direction
        if projDir.Norm()<0.000001:
            projectedVector = PyKDL.Vector(0,0,0)
        else:
            projectMag = PyKDL.dot(vector,projDir)
            # apply the magnitude on the direction
            projectedVector = projDir * projectMag
        return projectedVector

    @staticmethod
    def projectNullSpace(projDir,vector): 
        # this static method compute the projection of a vector
        if projDir.Norm()<0.000001:
            projectedVector = vector
        else:
            # onto the null space of a direction
            # Step ONE - find two unit vectors that represents the null space
            # we use two candidates(initializers) vectors to find the null space
            # checkout Gram-Schmidt on wikepedia

            candidateVect1 = PyKDL.Vector(1.0,1.0,1.0)
            candidateVect1.Normalize()
            candidateVect2 = PyKDL.Vector(0.5,-1.0,0.5)
            candidateVect2.Normalize()    
            if (abs(PyKDL.dot(candidateVect1,projDir))<\
                abs(PyKDL.dot(candidateVect2,projDir))):
                candidateVect = candidateVect1
            else:
                candidateVect = candidateVect2
            axis1 = candidateVect * projDir
            axis1.Normalize(); projDir.Normalize()
            axis2 = projDir * axis1 # think of projDir as z axis
            # Step TWO - find the prjection magnitude onto the two null space axes
            projMagAxis1 = PyKDL.dot(axis1,vector)
            projMagAxis2 = PyKDL.dot(axis2,vector)
            # Step THREE - apply the magnitudes to the axes and superimpose them
            projectedVector = \
            projMagAxis1*axis1 + projMagAxis2*axis2
        return projectedVector
if __name__ == '__main__':
    ContinuousPalpation(psmName = 'PSM1', forceTopic = '/atinetft/raw_wrench')

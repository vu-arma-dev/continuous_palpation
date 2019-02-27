#!/usr/bin/env python

# Continuous palpation of an organ
# Please note that the force notation is somewhat weird
# The "current force" is reported "on the robot"
# The "desired force" is reported in the direction 
# aligning with the desired motion associated with that force
# This means that if moving "down" into an organ, the 'current force' will be +z,
# whereas the "desired force direction" will be -z
# The wrist desired z direction is the same as the force desired direction


from collections import deque
import numpy as np
import rospy
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from std_msgs.msg import UInt32
from std_msgs.msg import Bool
import PyKDL
from tf_conversions import posemath
from tf import transformations
from dvrk import psm
import math
import rosparam

from time import time
import copy

from IPython import embed

# DEBUG TOOLS
import ipdb

class ContinuousPalpation:
    def __init__(self, psmName, forceTopic, bufferSize = 50):

        rospy.init_node('continuous_palpation', anonymous=True)
        
        self.fBuffer = deque([], bufferSize)
        self.fCurrent = PyKDL.Vector(0.0, 0.0, 0.0)

        self.trajectory = deque([])
        self.inContact = False
        self.pause = True
        self.lastCosVal=-1
        self.movingDirection=PyKDL.Vector(0,-1,0)
        # Set up subscibers
        self.poseSub = rospy.Subscriber(name = 'set_continuous_palpation_goal',
                                        data_class = PoseStamped,
                                        callback = self.poseCB,
                                        queue_size = 1)
        self.trajSub = rospy.Subscriber(name = 'set_continuous_palpation_trajectory',
                                        data_class = PoseArray,
                                        callback = self.poseArrayCB,
                                        queue_size = 1)
        self.forceSensorConfig = \
        {   'mount':'base' # either 'base' or 'tip', force sensor at the base of the phantom or at the tip of the robot
        }



        # Setup GP publishing data
        contPalpSpace = "/dvrk/"+psmName
        self.contactPublisher = rospy.Publisher(name = contPalpSpace + "/contact_state",
                                              data_class = Bool,
                                               queue_size = 1,
                                               latch = False)
        self.maxSinePublisher = rospy.Publisher(name = contPalpSpace + "/max_sine_event",
                                               data_class = Bool,
                                               queue_size = 1,
                                               latch = False)

        ################################################
        # IMPORTANT TODO
        # MAKE A PROPER ROTATION FOR THE INCOMING FORCES OR HAVE IT TAKEN CARE OF SEPARATELY BY ATINETFT
        ################################################
        self.FTRotate=True
        ################################################


        self.forceSub = rospy.Subscriber(name = forceTopic,
                                         data_class = WrenchStamped,
                                         callback = self.forceCB,
                                         queue_size = 1)


        # Set up publishers
        self.trajStatusPub = rospy.Publisher(name = 'trajectory_length',
                                             data_class = UInt32, 
                                             queue_size = 1,
                                             latch = False)

        self.robot = psm(psmName)
        self.robot.home()

        # TODO make these values not hard coded
        self.rate = rospy.Rate(1000) # 1000hz
        self.forceProfile = \
        {   'period': 1, # Seconds
            'amplitude': 2, # Newtons, default = 0.5
            'fBiasMag':  1, # Newtons, biased force magnitude, default=0.7
            'controlDir':'default', # 'default' or 'surf normal'
            'defaultDir':[0.0,0.0,-1.0], # default = [0.0,0.0,1.0]
            'admittanceGains':[ 55.0 / 1000,\
                                55.0 / 1000,\
                                55.0 / 1000], # force admittance gains, (m/s)/Newton
            'noiseThresh': 0.08, # Newtown, a threshold value to cancel the noise
            'b_forceOn': False,
        }

        self.resolvedRatesConfig = \
        {   'velMin': 1.0 / 1000,
            'velMax': 3.0 / 1000,
            'absVelMax': 7.0 / 1000,
            'velEpsilon': 1.5 / 1000,
            'angVelMin': 3.0 / 180.0 * np.pi,
            'angVelMax': 15.0 / 180.0 * np.pi,
            'tolPos': 0.5 / 1000, # positional tolerance
            'tolRot': 1.0 / 180*3.14, # rotational tolerance
            'velRatio': 3.0, # the ratio of max velocity error radius to
                              # tolarance radius, this value >1
            'rotRatio': 2.0,
            'dt': 1.0 / 1000, # this is the time step of the system. 
                            # if rate=1khz, then dt=1.0/1000. However, 
                            # we don't know if the reality will be the same
                            # as desired rate
            'b_wristOrient': True,
            'zref': PyKDL.Vector(0,0,-1),
            'coneSize': np.pi/3,
        }

        rospy.set_param('forceProfileInfo',self.forceProfile)
        rospy.set_param('resolvedRatesInfo',self.resolvedRatesConfig)
        rospy.set_param('moveDirInfo',self.movingDirection)

        self.trajIndex=1

        self.run()

    def run(self):
        commandedPose = self.robot.get_desired_position() # find start pose
        desiredPose = commandedPose # match original desired position at start
        tic=time()
        pauseTime=30
        while not rospy.is_shutdown():
            toc= (time()-tic)
            if np.floor(toc/pauseTime)==1:
                # ipdb.set_trace()
                tic=time()
            # Publish trajectory length at all times
            self.trajStatusPub.publish(self.trajIndex)

            # Check if there are any trajectories
            try:
                desiredPose = self.trajectory[0]
            except IndexError:
                # If no trajectory don't move, exit
                continue

            # get current and desired robot pose (desired is the top of queue)
            currentPose = self.robot.get_current_position() # this is used to capture the error

            # Update the wrist orientation so that the desired orientation matches the environment
            if self.resolvedRatesConfig['b_wristOrient'] and self.inContact:
                zcur=PyKDL.Vector(currentPose.M[0,2],currentPose.M[1,2],currentPose.M[2,2])
                zdes=-PyKDL.Vector(self.getAverageForce())
                zdes=self.coneLimit(zdes,self.resolvedRatesConfig['zref'],self.resolvedRatesConfig['coneSize'])

                zdes.Normalize()

                # Project the wrist orientation to only reorient in the moving direction
                # normalDirection=self.movingDirection*PyKDL.Vector(self.forceProfile['defaultDir'][0],self.forceProfile['defaultDir'][1],self.forceProfile['defaultDir'][2])
                # zdes = self.projectNullSpace2(normalDirection,zdes)

                axis = zcur*zdes
                crossNorm=axis.Normalize()
                dotResult=PyKDL.dot(zcur,zdes)
                angle = math.atan2(crossNorm,dotResult)

                diffTwist=PyKDL.Twist(PyKDL.Vector(),axis*angle)
                desiredPose=PyKDL.addDelta(desiredPose,diffTwist,1)
                pass

            # compute the desired twist "x_dot" from motion command [PyKDL.Twist]
            [xDotMotion, goalPoseReached] = self.resolvedRates(currentPose, \
                                                               desiredPose)
            # Do hybrid force motion if using the force controller
            if self.forceProfile['b_forceOn']:
                # compute the desired twist "x_dot" from force command [PyKDL.Twist]

                if self.resolvedRatesConfig['b_wristOrient'] and self.inContact:
                    try:
                        forceCtrlDir = zdes
                    except NameError:
                        forceCntrlDir = PyKDL.Vector(\
                            self.forceProfile['defaultDir'][0],
                            self.forceProfile['defaultDir'][1],
                            self.forceProfile['defaultDir'][2])
                else:
                    forceCtrlDir = self.updateForceControlDir()
                xDotForce = self.forceAdmittanceControl(forceCtrlDir)
                
                [xDot, goalPoseReached] = self.hybridPosForce(xDotMotion,
                                                               xDotForce,
                                                               forceCtrlDir)
            else:
                xDot=xDotMotion
            # Check whether we have reached our goal
            if goalPoseReached:
               # ipdb.set_trace()
               # When first reached the first goal pose, turn on the force controller
               self.forceProfile['b_forceOn']=True
               if len(self.trajectory)>1 and self.inContact:
                    # self.movingDirection=PyKDL.Vector((PyKDL.diff(self.trajectory[1],self.trajectory[0])).vel)
                    # self.movingDirection.Normalize()
                    self.trajectory.popleft()
                    self.trajIndex=self.trajIndex+1
                    print("Trajectories finished: "+str(self.trajIndex))
                    pass
                   
               # else:
                    # ipdb.set_trace()
            # apply the desired twist on the currnet pose
            dt = self.resolvedRatesConfig['dt']
            commandedPose = PyKDL.addDelta(commandedPose, xDot, dt)
            #ipdb.set_trace()
            # Move the robot
            self.robot._arm__move_frame(commandedPose, interpolate = False)
            self.rate.sleep()

    def forceCB(self, data):
        # The received data needs to be in PyKDL.Vector format
        force = data.wrench.force
        if self.FTRotate:
            self.fCurrent = PyKDL.Vector(-force.x,-force.y,force.z)
        else:
            self.fCurrent = PyKDL.Vector(force.x,force.y,force.z)
        if self.forceSensorConfig['mount'] == 'base':
            self.fCurrent = -self.fCurrent

        self.fBuffer.append(self.fCurrent)
        self.inContact = self.fCurrent.Norm() > 0.15

        self.contactPublisher.publish(Bool(self.inContact))
        # if self.inContact and self.pause:
        #     ipdb.set_trace()
    
    def getAverageForce(self):
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

    def poseArrayCB(self, data):
        # Check if timestamp makes sense
        time_diff = (rospy.get_rostime() - data.header.stamp).to_sec()
        if time_diff > 10:
            print("Ignoring old message timestamped %.2fs ago" % time_diff)
            return
        else:
            print("Accepting a message")
        # Fill out pose array
        for pose in data.poses:
            self.trajectory.append(posemath.fromMsg(pose))
            print("Trajectory now length "+ str(len(self.trajectory)))

    def resolvedRates(self,currentPose,desiredPose):
        # compute pose error (result in kdl.twist format)
        poseError = PyKDL.diff(currentPose,desiredPose)
        posErrNorm = poseError.vel.Norm()
        rotErrNorm = poseError.rot.Norm()
        # compute velocity magnitude based on position error norm
        if posErrNorm > self.resolvedRatesConfig['tolPos']:
            tolPosition = self.resolvedRatesConfig['tolPos']
            lambdaVel = self.resolvedRatesConfig['velRatio']
            if posErrNorm > (lambdaVel * tolPosition):
                velMag = self.resolvedRatesConfig['velMax']
            else:
                velMax = self.resolvedRatesConfig['velMax']
                velMin = self.resolvedRatesConfig['velMin']
                velMag = (velMin \
                          + (posErrNorm - tolPosition) \
                          * (velMax-velMin) / (tolPosition * (lambdaVel-1)))
        else:
            velMag = 0.0
        # compute angular velocity based on rotation error norm
        if rotErrNorm > self.resolvedRatesConfig['tolRot']:
            tolRotation = self.resolvedRatesConfig['tolRot']
            lambdaRot = self.resolvedRatesConfig['rotRatio']
            if rotErrNorm > (lambdaRot * tolRotation):
                angVelMag = self.resolvedRatesConfig['angVelMax']
            else:
                angVelMax = self.resolvedRatesConfig['angVelMax']
                angVelMin = self.resolvedRatesConfig['angVelMin']
                angVelMag = (angVelMin \
                             + (rotErrNorm - tolRotation) \
                             * (angVelMax - angVelMin) \
                             / (tolRotation * (lambdaRot-1)))
        else:
            angVelMag = 0.0
            # The resolved rates is implemented as Nabil Simaan's notes
        # apply both the velocity and angular velocity in the error pose direction
        desiredTwist = PyKDL.Twist()
        poseError.vel.Normalize() # normalize to have the velocity direction
        desiredTwist.vel = poseError.vel * velMag
        poseError.rot.Normalize() # normalize to have the ang vel direction
        desiredTwist.rot = poseError.rot * angVelMag
        # Check whether we have reached our goal
        # goalReached = desiredTwist.vel.Norm() <= self.resolvedRatesConfig['velMin'] \
                      # and desiredTwist.rot.Norm() <= self.resolvedRatesConfig['angVelMin']
        
        # Ignore rotation for calculating goalReached!
        goalReached = desiredTwist.vel.Norm() <= self.resolvedRatesConfig['velMin'] 
        return desiredTwist, goalReached

    def updateForceControlDir(self):
        # this func updates the force control direction based on specs
        # load the desired force control direction according to different mode
        if self.forceProfile['controlDir'] == 'surf normal' and self.inContact:
            # under this condition, 
            # need to compute the force control direction based on f_buffer
            normalDirection=self.movingDirection*PyKDL.Vector(self.forceProfile['defaultDir'][0],self.forceProfile['defaultDir'][1],self.forceProfile['defaultDir'][2])
            forceCtrlDir = self.projectNullSpace2(normalDirection,-PyKDL.Vector(self.getAverageForce()))


            # Need to check the norm of the force readings, if too small, treat it as noise
            if (forceCtrlDir.Norm() > self.forceProfile['noiseThresh']):
                forceCtrlDir.Normalize()
            else:
                forceCtrlDir = PyKDL.Vector(\
                    self.forceProfile['defaultDir'][0],
                    self.forceProfile['defaultDir'][1],
                    self.forceProfile['defaultDir'][2])                        
        else: 
            # if not in contact/not using surf normal, use default direction
            forceCtrlDir = PyKDL.Vector(\
                self.forceProfile['defaultDir'][0],
                self.forceProfile['defaultDir'][1],
                self.forceProfile['defaultDir'][2])
        return forceCtrlDir

    def forceAdmittanceControl(self,forceCtrlDir):
        # compute the desired force magnitude  
        curTime = (2 * np.pi)/self.forceProfile['period'] * time()  
        sinamplitudeMag = np.sin(curTime) * abs(self.forceProfile['amplitude'])/2+ abs(self.forceProfile['amplitude'])/2
        cosVal = np.cos(curTime)

        if cosVal<=0 and self.lastCosVal>=0:
            self.maxSinePublisher.publish(Bool(True))
        self.lastCosVal=cosVal


        fRefMag = sinamplitudeMag + abs(self.forceProfile['fBiasMag'])
        # compute desired force
        desiredForce = forceCtrlDir * fRefMag
        # get the current force
        forceError = self.fCurrent + desiredForce
        # apply admittance gain to compute motion
        desiredTwist = PyKDL.Twist()
        desiredTwist.vel = PyKDL.Vector(\
            forceError.x()*self.forceProfile['admittanceGains'][0],
            forceError.y()*self.forceProfile['admittanceGains'][1],
            forceError.z()*self.forceProfile['admittanceGains'][2])
        velNorm = desiredTwist.vel.Norm()
        if velNorm > self.resolvedRatesConfig['absVelMax']:
            desiredTwist.vel.Normalize()
            desiredTwist.vel =  desiredTwist.vel \
                                * self.resolvedRatesConfig['absVelMax']
        desiredTwist.rot = PyKDL.Vector(0.0,0.0,0.0)
        return desiredTwist

    def hybridPosForce(self, xDotMotion, xDotForce, forceCtrlDir):
        # project the force command onto the force control direction
        velForce = self.projectDirection(forceCtrlDir,xDotForce.vel)
        xDotForce.vel = velForce
        # project the motion command onto the null space of force control direction
        #velMotion = self.projectNullSpace(forceCtrlDir,xDotMotion.vel)
        velMotion = self.projectNullSpace2(forceCtrlDir,xDotMotion.vel)
        xDotMotion.vel = velMotion
        # combine twist 
        xDot = xDotMotion + xDotForce

        # Check whether we have reached our goal
        # goalReached = xDotMotion.vel.Norm() <= self.resolvedRatesConfig['velMin'] \
        #               and xDotMotion.rot.Norm() <= self.resolvedRatesConfig['angVelMin']
        goalReached = xDotMotion.vel.Norm()  <= self.resolvedRatesConfig['velMin']  \
                        and xDotForce.vel.Norm() <= self.resolvedRatesConfig['velEpsilon']
        return xDot, goalReached

    def projectNullSpace2(self,projDir,vector):
        # this static method computes vector projection in a different way
        return vector-self.projectDirection(projDir,vector)

    @staticmethod
    def projectDirection(projDir,vector):
        # this static method compute the projection of a vector onto a direction
        # both variables need to be in format of PyKDL.Vector
        # compute the projection magnitude of vector onto the direction
        if projDir.Norm() < 0.000001:
            projectedVector = PyKDL.Vector(0,0,0)
        else:
            projectMag = PyKDL.dot(vector,projDir)
            # apply the magnitude on the direction
            projectedVector = projDir * projectMag
        return projectedVector

    @staticmethod
    def coneLimit(zcur,zref,coneAngle):
        zcur.Normalize()
        zref.Normalize()
        dotResult = PyKDL.dot(zcur,zref)
        offset_angle = coneAngle/2;

        if dotResult < np.cos(offset_angle):
            axis=zref* zcur
            axis.Normalize()

            # ipdb.set_trace()
            R=PyKDL.Rotation().Rot(axis,offset_angle)
            zcur=R*zref
        # ipdb.set_trace()
        return zcur

    @staticmethod
    def projectNullSpace(projDir,vector): 
        # this static method compute the projection of a vector
        if projDir.Norm() < 0.00001:
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
            if abs(PyKDL.dot(candidateVect1,projDir)) < abs(PyKDL.dot(candidateVect2,projDir)):
                candidateVect = candidateVect1
            else:
                candidateVect = candidateVect2
            axis1 = candidateVect * projDir
            axis1.Normalize()
            projDir.Normalize()
            axis2 = projDir * axis1 # think of projDir as z axis
            # Step TWO - find the prjection magnitude onto the two null space axes
            projMagAxis1 = PyKDL.dot(axis1,vector)
            projMagAxis2 = PyKDL.dot(axis2,vector)
            # Step THREE - apply the magnitudes to the axes and superimpose them
            projectedVector = projMagAxis1 * axis1 + projMagAxis2 * axis2
        return projectedVector

    @staticmethod
    def Rot2Mat(R):
        ar = np.array([[R[0]],[],[]])
        return ar


if __name__ == '__main__':
    ContinuousPalpation(psmName = 'PSM2', forceTopic = '/dvrk/PSM2/wrench')
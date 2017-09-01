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

# DEBUG TOOLS
import ipdb

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
        self.forceProfile = \
        {   'period':0.5, # Seconds
            'amplitude': 0.5, # Newtons
            'fBiasMag': 0.7, # Newtons, biased force magnitude
            'magnitudeMode': 'bias', # 'bias' or 'sine', or 'sine bias'
            'controlDir':'default', # 'default' or 'surf normal'
            'defaultDir':[0.0,0.0,1.0],
            'admittanceGains':[ 100.0/1000,\
                                100.0/1000,\
                                100.0/1000] # force admittance gains, (meter/sec)/Newton
        }
        self.resolvedRatesConfig = \
        {   'velMin': 5.0/1000,
            'velMax': 50.0/1000,
            'angVelMin': 3.0/180.0*3.14,
            'angVelMax': 60.0/180.0*3.14,
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
            try:
                ipdb.set_trace()
                nextPose = self.trajectory[0]
            except IndexError:
                # If no trajectory do nothing
                continue
            xDotMotion = self.resolvedRates() # xDotMotion is type [PyKDL.Twist]
            xDotForce = self.forceAdmittanceControl() # xDotForce is type [PyKDL.Twist]
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
    
    def getAverageForce(self):
	npAvg = np.mean(f_buffer,0)
        fAverage = PyKDL.Vector(npAvg[0],npAvg[1],npAvg[2])
        return f_average
    
    def poseCB(self, data):
        self.trajectory.append(posemath.fromMsg(data.pose))
    
    def poseArrayCB(self, data):
        for pose in data.poses:
            self.trajectory.append(posemath.fromMsg(pose))

    def resolvedRates(self):
        # get current and desired robot pose (desired is the top of queue)
        currentPose = self.robot.get_current_position()
        desiredPose = self.trajectory[0]
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
                velMag = velMin + \
                (velMax - velMin)/(tolPosition*(lambdaVel-1))
        else:
            velMag = 0.0
        # compute angular velocity based on rotation error norm
        if rotErrNorm>self.resolvedRatesConfig['tolRot']:
            tolRotation = self.resolvedRatesConfig['tolRot']
            lambdaRot = self.resolvedRatesConfig['rotRatio']
            if posErrNorm>(lambdaRot*tolRotation):
                angVelMag = self.resolvedRatesConfig['angVelMax']
            else:
                angVelMax = self.resolvedRatesConfig['angVelMax']
                angVelMin = self.resolvedRatesConfig['angVelMin']
                angVelMag = angVelMin + \
                (angVelMax - angVelMin)/(tolRotation*(lambdaRot-1))
        else:
            angVelMag = 0.0
            # The resolved rates is implemented as Nabil Simaan's notes
        # apply both the velocity and angular velocity in the error pose direction
        sys_dt = self.resolvedRatesConfig['dt']
        desiredTwist = PyKDL.Twist()
        poseError.vel.Normalize() # normalize to have the velocity direction
        desiredTwist.vel = poseError.vel*velMag*sys_dt
        poseError.rot.Normalize() # normalize to have the ang vel direction
        desiredTwist.rot = poseError.rot*angVelMag*sys_dt
        return desiredTwist
	
    def computeFRef(self):
        if self.forceProfile['magnitudeMode']=='bias':
            force_ref = self.forceProfile['fBiasMag']
        elif self.forceProfile['magnitudeMode']=='sine':
            force_ref = \
            np.sin(self.forceProfile['period']*time() / (2*np.PI)) * \
            self.forceProfile['amplitude']
        else:
            sine_mag = \
            np.sin(self.forceProfile['period']*time() / (2*np.PI)) * \
            self.forceProfile['amplitude']
            force_ref = sine_mag + self.forceProfile['fBiasMag']
        return force_ref

    def forceAdmittanceControl(self):
        # load the desired force control direction according to different mode
        if self.forceProfile['controlDir'] == 'default':
            force_Ctrl_Dir = PyKDL.Vector(\
                self.forceProfile['defaultDir'][0],
                self.forceProfile['defaultDir'][1],
                self.forceProfile['defaultDir'][2])
        else: 
            # under this condition, 
            # need to compute the force control direction based on f_buffer
            force_Ctrl_Dir = self.getAverageForce()
        force_Ctrl_Dir.Normalize()
        # compute the desired force magnitude  
        f_ref_mag = self.computeFRef()
        # compute desired force
        desiredForce = force_Ctrl_Dir * f_ref_mag
        # apply admittance gain to compute motion
        sys_dt = self.resolvedRatesConfig['dt']
        desiredTwist = PyKDL.Twist()
        desiredTwist.vel = PyKDL.Vector(\
            desiredForce.x()*self.forceProfile['admittanceGains'][0]*sys_dt,
            desiredForce.y()*self.forceProfile['admittanceGains'][1]*sys_dt,
            desiredForce.z()*self.forceProfile['admittanceGains'][2]*sys_dt) 
        return desiredTwist

    def hybridPosForce(self, xDotMotion, xDotForce, poseCur):
        # TODO implement hybrid force position control
        xDot = xDotForce + xDotMotion
        return xDot * (1.0/self.rate) + poseCur

    def checkEqual(self, current, desired):
	# TODO check equality before popping
	try:
	    self.trajectory.popleft()
	except IndexError:
	    # TODO handle end of function
            print "no more trajectories"

if __name__ == '__main__':
    ContinuousPalpation(psmName = 'PSM1', forceTopic = '/atinetft/raw_wrench')

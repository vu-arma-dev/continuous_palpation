#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from std_msgs.msg import UInt32
from collections import deque
# from dvrk import psm

class ContinuousPalpation:
    def __init__(self, psmName, forceTopic, bufferSize = 50):

        rospy.init_node('continuous_palpation', anonymous=True)
        
        self.f_buffer = deque([], bufferSize)
        self.f_current = np.zeros((3,1))

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
        trajStatusPub = rospy.Publisher(name = 'trajectory_length',
                                        data_class = UInt32, 
                                        queue_size = 1)

        # robot = psm(psmName)

        rate = rospy.Rate(1000) # 1000hz
        while not rospy.is_shutdown():
            trajStatusPub.publish(len(self.trajectory))
            rate.sleep()
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    def forceCB(self, data):
        self.f_current = np.array((data.wrench.force.x,
                                   data.wrench.force.y,
                                   data.wrench.force.z))
        self.f_buffer.append(f_current)

    def poseCB(self, data):
        self.trajectory.append( np.array((data.pose.position.x,
                                          data.pose.position.y,
                                          data.pose.position.z,
                                          data.pose.orientation.x,
                                          data.pose.orientation.y,
                                          data.pose.orientation.z,
                                          data.pose.orientation.w)))
    
    def poseArrayCB(self, data):
        for pose in data.poses:
            self.trajectory.append( np.array((pose.position.x,
                                              pose.position.y,
                                              pose.position.z,
                                              pose.orientation.x,
                                              pose.orientation.y,
                                              pose.orientation.z,
                                              pose.orientation.w)))

if __name__ == '__main__':
    ContinuousPalpation(psmName = 'PSM1', forceTopic = '/atinetft/raw_wrench')
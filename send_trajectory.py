from IPython import embed
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from tf_conversions import posemath
import PyKDL
import rospy
from dvrk import psm
import numpy as np
import ipdb

posePub = rospy.Publisher(name = 'set_continuous_palpation_trajectory',
                          data_class = PoseArray, 
                          queue_size = 1)

def send_trajectory(positions, robot):
    ''' 
    SEND_TRAJECTORY Sends a trajectory message to a continuous palpation node
        Positions is a list of PyKDL vectors. The current orientation is kept
        constant.
    '''
    msg = PoseArray()
    msg.header.stamp = rospy.Time.now()
    for idx in range(0,len(positions)):
        pose = posemath.toMsg(robot.get_current_position())
        pose.position.x = positions[idx].x()
        pose.position.y = positions[idx].y()
        pose.position.z = positions[idx].z()
        msg.poses.append(pose)
    posePub.publish(msg)

def send_trajectory_relative(positions, robot):
    ''' 
    SEND_TRAJECTORY Sends a trajectory message to a continuous palpation node
        Positions is a list of PyKDL vectors. The current orientation is kept
        constant.
    '''

    msg = PoseArray()
    msg.header.stamp = rospy.Time.now()
    for idx in range(0,len(positions)):
        pose = posemath.toMsg(robot.get_current_position())
        pose.position.x = pose.position.x + positions[idx].x()
        pose.position.y = pose.position.y + positions[idx].y()
        pose.position.z = pose.position.z + positions[idx].z()
        msg.poses.append(pose)
    posePub.publish(msg)


if __name__ == '__main__':
    robot = psm("PSM1")
    embed()
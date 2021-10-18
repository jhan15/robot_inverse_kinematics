#! /usr/bin/env python

"""
    This file publishes a square trajectory for the kuka
"""

from scara_square_trajectory_publisher import SquareTrajectoryPublisher
import rospy
import rospkg
rospack = rospkg.RosPack()
import sys
sys.path.insert(0, rospack.get_path('first_assignment')+"/scripts")
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseArray
from IK_function import scara_IK
import numpy as np


def main():
    rospy.init_node('trajectory_publisher')
    topic_name = rospy.get_param('topic_name', 'controller/joint_states')
    rate = rospy.Rate(10)
    trajectory_publisher = SquareTrajectoryPublisher()
    trajectory_publisher.set_topic_name(topic_name)
    trajectory_publisher._joint_names = ['lwr_a1_joint', 'lwr_a2_joint', 'lwr_e1_joint', 'lwr_a3_joint', 
                                'lwr_a4_joint', 'lwr_a5_joint', 'lwr_a6_joint']

    trajectory_publisher._path.header.frame_id = 'lwr_base_link'

    while not rospy.is_shutdown():
        trajectory_publisher.publish_path()
        trajectory_publisher.execute_step()
        rate.sleep()


if __name__ == '__main__':
    main()
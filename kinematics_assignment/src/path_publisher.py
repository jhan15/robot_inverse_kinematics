#! /usr/bin/env python

"""
    This node publishes the desired path for the SCARA robot

    @author: Silvia Cruciani (cruciani@kth.se)
"""

import rospy
from scara_square_trajectory_publisher import SquareTrajectoryPublisher

if __name__ == '__main__':
    rospy.init_node('path_publisher')
    rate = rospy.Rate(10)
    trajectory_publisher = SquareTrajectoryPublisher()
    while not rospy.is_shutdown():
        trajectory_publisher.publish_path()
        rate.sleep()
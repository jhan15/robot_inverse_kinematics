#! /usr/bin/env python

"""
    This node publishes the joint states to make a square trajectory with the SCARA's end-effector

    @author: Silvia Cruciani (cruciani@kth.se)
"""

import rospy
from square_trajectory import SquareTrajectory
import IK_functions
from sensor_msgs.msg import JointState
from std_srvs.srv import EmptyResponse, EmptyRequest, Empty


def main():
    rospy.init_node('scara_node')
    rate = rospy.Rate(10)
    #the 4 vertices of the square
    vertices = [[0.27, -0.15, 0], [0.57, -0.15, 0.1], [0.57, 0.15, 0.1], [0.27, 0.15, 0]]
    #the name of the robot's base frame
    base_frame = 'base'
    trajectory_publisher = SquareTrajectory(vertices, base_frame)

    #the joint names of scara:
    joint_names = ['rotational1', 'rotational2', 'translation']
    #define the ros message for publishing the joint positions
    joint_msg = JointState()
    joint_msg.name = joint_names

    #define the ros topic where to publish the joints values
    topic_name = rospy.get_param('~topic_name', 'controller/joint_states')
    publisher = rospy.Publisher(topic_name, JointState, queue_size=10)

    def restart(req):
        reload(IK_functions)
        trajectory_publisher.restart()
        trajectory_publisher.publish_path()
        rate.sleep()

        return EmptyResponse()

    s = rospy.Service('restart', Empty, restart)

    restart(EmptyRequest())

    while not rospy.is_shutdown():
        #get the current point in the trajectory
        point = trajectory_publisher.get_point()
        if point is not None:
            #get the IK solution for this point
            q = IK_functions.scara_IK(point)
            #publish this solution
            joint_msg.position = q
            publisher.publish(joint_msg)

        #publish the path to be visualized in rviz
        trajectory_publisher.publish_path()
        rate.sleep()


if __name__ == '__main__':
    main()
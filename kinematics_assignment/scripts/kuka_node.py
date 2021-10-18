#! /usr/bin/env python

"""
    This node publishes the joint states to make a given trajectory with the KUKA's end-effector

    @author: Silvia Cruciani (cruciani@kth.se)
"""

import rospy
from square_trajectory import SquareTrajectory
import IK_functions
from sensor_msgs.msg import JointState
from std_srvs.srv import EmptyResponse, EmptyRequest, Empty

def main():
    rospy.init_node('kuka_node')
    rate = rospy.Rate(10)
    #the vertices of the square trajectory (in this case it will be a line)
    vertices = [[-0.217, 0, 0.84], [-0.2, 0, 0.65], [-0.2, 0, 0.65], [-0.217, 0, 0.84]]
    #the name of the robot's base frame
    base_frame = 'lwr_base_link'

    trajectory_publisher = SquareTrajectory(vertices, base_frame)
    desired_orientation = [[0, 0, -1], [0, 1, 0], [1, 0, 0]]

    current_q = [0, 1.12, 0, 1.71, 0, 1.84, 0]

    #the joint names of kuka:
    joint_names = ['lwr_a1_joint', 'lwr_a1_joint_stiffness', 'lwr_a2_joint', 'lwr_a2_joint_stiffness', 'lwr_e1_joint', 
    'lwr_e1_joint_stiffness', 'lwr_a3_joint', 'lwr_a3_joint_stiffness', 'lwr_a4_joint', 'lwr_a4_joint_stiffness',
    'lwr_a5_joint', 'lwr_a5_joint_stiffness', 'lwr_a6_joint', 'lwr_a6_joint_stiffness']
    #define the ros message for publishing the joint positions
    joint_msg = JointState()
    joint_msg.name = joint_names

    #define the ros topic where to publish the joints values
    topic_name = rospy.get_param('~topic_name', 'controller/joint_states')
    publisher = rospy.Publisher(topic_name, JointState, queue_size=10)

    # A service is used to restart the trajectory execution and reload a new solution to test
    def restart(req):
        reload(IK_functions)
        current_q[0:7] = [0, 1.12, 0, 1.71, 0, 1.84, 0]
        q_msg = [current_q[0], 0, current_q[1], 0, current_q[2], 0, current_q[3], 0, current_q[4], 0, current_q[5], 0, current_q[6], 0]
        joint_msg.position = q_msg
        publisher.publish(joint_msg)
        trajectory_publisher.restart()
        trajectory_publisher.publish_path()
        rate.sleep()

        return EmptyResponse()

    s = rospy.Service('restart', Empty, restart)

    restart(EmptyRequest())

    rate.sleep()

    while not rospy.is_shutdown():
        #get the current point in the trajectory
        point = trajectory_publisher.get_point()
        if point is not None:
            #get the IK solution for this point
            q = IK_functions.kuka_IK(point, desired_orientation, current_q)
            current_q = q
            q_msg = [q[0], 0, q[1], 0, q[2], 0, q[3], 0, q[4], 0, q[5], 0, q[6], 0]
            #publish this solution
            joint_msg.position = q_msg
            publisher.publish(joint_msg)

        #publish the path to be visualized in rviz
        trajectory_publisher.publish_path()
        rate.sleep()


if __name__ == '__main__':
    main()
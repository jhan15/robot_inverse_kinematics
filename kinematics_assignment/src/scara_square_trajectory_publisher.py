#! /usr/bin/env python

"""
    This node publishes the joint states to make a square trajectory with the SCARA's end-effector

    @author: Silvia Cruciani (cruciani@kth.se)
"""

import rospy
import rospkg
rospack = rospkg.RosPack()
import sys
sys.path.insert(0, rospack.get_path('first_assignment')+"/scripts")
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseArray
from IK_function import scara_IK
import numpy as np

class SquareTrajectoryPublisher():
    """docstring for SquareTrajectoryPublisher"""
    def __init__(self):
        self._publisher = None
        self._path_publisher = rospy.Publisher('desired_path', PoseArray, queue_size=10)
        self._path = PoseArray()
        self._path.header.frame_id = 'base'
        self._dt = 0.1
        self._v = 0.05
        #the 4 vertices of the square
        self._vertices = [ [0.27, -0.15, 0], 
                            [0.57, -0.15, 0.1],
                            [0.57, 0.15, 0.1],
                            [0.27, 0.15, 0]  ]

        self._joint_names = ['rotational1', 'rotational2', 'translation']

        self._current_segment = 0
        self._current_idx = 0
        self._waypoints = None
        self.compute_waypoints()

    def set_topic_name(self, name):
        self._publisher = rospy.Publisher(name, JointState, queue_size=10)

    def next_segment(self):
        if self._current_segment is 3:
            self._current_segment = 0
        else:
            self._current_segment += 1

        self._current_idx = 0

    def return_list_of_waypoints(self, p1, p2, dp, it):
        waypoints = list()
        current_p = p1
        waypoints.append(current_p)
        p = Pose()
        p.position.x = current_p[0]
        p.position.y = current_p[1]
        p.position.z = current_p[2]
        p.orientation.w = 0.707
        p.orientation.y = 0.707
        self._path.poses.append(p)
        for i in range(1, int(abs(it))):
            current_p = current_p + dp
            waypoints.append(current_p)
            p = Pose()
            p.position.x = current_p[0]
            p.position.y = current_p[1]
            p.position.z = current_p[2]
            p.orientation.w = 0.707
            p.orientation.y = 0.707
            self._path.poses.append(p)

        waypoints.append(p2)

        return waypoints


    def compute_waypoints(self):
        ds = self._v*self._dt
        v1 = np.array(self._vertices[0])
        v2 = np.array(self._vertices[1])
        v3 = np.array(self._vertices[2])
        v4 = np.array(self._vertices[3])

        v = v2 - v1
        v_ds = v/ds
        it = abs(v_ds[(np.absolute(v_ds)).argmax()])
        dv = v/float(it)
        v1v2 = self.return_list_of_waypoints(v1, v2, dv, it)

        v = v3 - v2
        v_ds = v/ds
        it = abs(v_ds[(np.absolute(v_ds)).argmax()])
        dv = v/float(it)
        v2v3 = self.return_list_of_waypoints(v2, v3, dv, it)


        v = v4 - v3
        v_ds = v/ds
        it = abs(v_ds[(np.absolute(v_ds)).argmax()])
        dv = v/float(it)
        v3v4 = self.return_list_of_waypoints(v3, v4, dv, it)


        v = v1 - v4
        v_ds = v/ds
        it = abs(v_ds[(np.absolute(v_ds)).argmax()])
        dv = v/float(it)
        v4v1 = self.return_list_of_waypoints(v4, v1, dv, it)

        self._waypoints = [v1v2, v2v3, v3v4, v4v1]

    def send_joint_position(self, position):
        m = JointState()
        m.name = self._joint_names
        m.position = position

        self._publisher.publish(m)

    def publish_path(self):
        self._path_publisher.publish(self._path)



        
    def execute_step(self):
        l = len(self._waypoints[self._current_segment])
        if(self._current_idx >= l):
            self.next_segment()
        
        desired_point = (self._waypoints[self._current_segment])[self._current_idx]
        self._current_idx += 1

        q = scara_IK(desired_point)
        self.send_joint_position(q)





        



def main():
    rospy.init_node('trajectory_publisher')
    topic_name = rospy.get_param('topic_name', 'controller/joint_states')
    rate = rospy.Rate(10)
    trajectory_publisher = SquareTrajectoryPublisher()
    trajectory_publisher.set_topic_name(topic_name)
    while not rospy.is_shutdown():
        trajectory_publisher.execute_step()
        # trajectory_publisher.publish_path()
        rate.sleep()


if __name__ == '__main__':
    main()
        
#! /usr/bin/env python

"""
    This class contains a squared trajectory in 3D given the 4 vertices, and it publishes in rviz

    @author: Silvia Cruciani (cruciani@kth.se)
"""

import rospy
import rospkg
rospack = rospkg.RosPack()
from geometry_msgs.msg import Pose, PoseArray
import numpy as np

class SquareTrajectory():
    """docstring for SquareTrajectory"""
    def __init__(self, vertices=[[0.27, -0.15, 0], [0.57, -0.15, 0.1], [0.57, 0.15, 0.1], [0.27, 0.15, 0]], base_frame='base'):
        self._path_publisher = rospy.Publisher('desired_path', PoseArray, queue_size=10)
        self._path = PoseArray()
        self._path.header.frame_id = base_frame
        self._dt = 0.1
        self._v = 0.05
        #the 4 vertices of the square
        self._vertices = vertices

        self._current_segment = 0
        self._current_idx = 0
        self._waypoints = None
        self.compute_waypoints()


    def next_segment(self):
        """ This function returns the nex segment of the square. -1 if the path ended """
        if self._current_segment is 3:
            self._current_segment = -1
        else:
            self._current_segment += 1

        self._current_idx = 0

    def return_list_of_waypoints(self, p1, p2, dp, it):
        """ This function returns a list of waypoints between a start and a goal point """
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
        """ This function computes all the 4 segments of the square, given the initial vertices """
        ds = self._v*self._dt
        v1 = np.array(self._vertices[0])
        v2 = np.array(self._vertices[1])
        v3 = np.array(self._vertices[2])
        v4 = np.array(self._vertices[3])

        v = v2 - v1
        v_ds = v/ds
        it = abs(v_ds[(np.absolute(v_ds)).argmax()])
        if abs(float(it) < 0.000001):
            v1v2 = []
        else:
            dv = v/float(it)
            v1v2 = self.return_list_of_waypoints(v1, v2, dv, it)

        v = v3 - v2
        v_ds = v/ds
        it = abs(v_ds[(np.absolute(v_ds)).argmax()])
        if abs(float(it) < 0.000001):
            v2v3 = []
        else:
            dv = v/float(it)
            v2v3 = self.return_list_of_waypoints(v2, v3, dv, it)


        v = v4 - v3
        v_ds = v/ds
        it = abs(v_ds[(np.absolute(v_ds)).argmax()])
        if abs(float(it) < 0.000001):
            v3v4 = []
        else:
            dv = v/float(it)
            v3v4 = self.return_list_of_waypoints(v3, v4, dv, it)


        v = v1 - v4
        v_ds = v/ds
        it = abs(v_ds[(np.absolute(v_ds)).argmax()])
        if abs(float(it) < 0.000001):
            v4v1 = []
        else:
            dv = v/float(it)
            v4v1 = self.return_list_of_waypoints(v4, v1, dv, it)

        self._waypoints = [v1v2, v2v3, v3v4, v4v1]

    def publish_path(self):
        """ This function publishes the path in rviz"""
        self._path_publisher.publish(self._path)

        
    def get_point(self):
        """ This function returns the next point in the path. None if the path ended """
        if self._current_segment is -1:
            return None
        l = len(self._waypoints[self._current_segment])
        if(self._current_idx >= l):
            self.next_segment()
        
        if self._current_segment is -1:
            return None

        if len(self._waypoints[self._current_segment]) < 1:
            self.next_segment()

        if self._current_segment is -1:
            return None
        
        desired_point = (self._waypoints[self._current_segment])[self._current_idx]
        self._current_idx += 1

        return desired_point

    def restart(self):
        """ This function resets the current point to go through the path once again """
        self._current_segment = 0
        self._current_idx = 0
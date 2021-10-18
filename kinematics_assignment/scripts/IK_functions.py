#! /usr/bin/env python3
import math
import numpy as np
from math import sin, cos



"""
    # {Jianming Han}
    # {jiahan@kth.se}
"""

def scara_IK(point):
    x = point[0]
    y = point[1]
    z = point[2]
    q = [0.0, 0.0, 0.0]

    """
    Fill in your IK solution here and return the three joint values in q
    """

    # initialize robot links
    l = [0.07, 0.3, 0.35]

    # calculate robot configuration
    cos_2 = ((x-l[0])**2 + y**2 - l[1]**2 - l[2]**2) / (2*l[1]*l[2])
    sin_2 = math.sqrt(1 - cos_2**2)

    q[0] = math.atan2(y, x-l[0]) - math.atan2(l[2]*sin_2, l[1]+l[2]*cos_2)
    q[1] = math.atan2(sin_2, cos_2)
    q[2] = z

    return q

def kuka_IK(point, R, joint_positions):
    x = point[0]
    y = point[1]
    z = point[2]
    q = joint_positions #it must contain 7 elements

    """
    Fill in your IK solution here and return the seven joint values in q
    """

    # initialize robot links
    k = 0.311
    l = 0.4
    m = 0.39
    n = 0.078

    # manipulate tolerance
    tolerance = 0.001

    # set initial error to a big value
    error_end = 100


    # repeat until error <= tolerance
    while error_end > tolerance:
        # calculate transformation
        t10 = [
            [cos(q[0]), 0, sin(q[0]), 0],
            [sin(q[0]), 0, -cos(q[0]), 0],
            [0, 1, 0, 0],
            [0, 0, 0, 1]
        ]
        t21 = [
            [cos(q[1]), 0, -sin(q[1]), 0],
            [sin(q[1]), 0, cos(q[1]), 0],
            [0, -1, 0, 0],
            [0, 0, 0, 1]
        ]
        t32 = [
            [cos(q[2]), 0, -sin(q[2]), 0],
            [sin(q[2]), 0, cos(q[2]), 0],
            [0, -1, 0, l],
            [0, 0, 0, 1]
        ]
        t43 = [
            [cos(q[3]), 0, sin(q[3]), 0],
            [sin(q[3]), 0, -cos(q[3]), 0],
            [0, 1, 0, 0],
            [0, 0, 0, 1]
        ]
        t54 = [
            [cos(q[4]), 0, sin(q[4]), 0],
            [sin(q[4]), 0, -cos(q[4]), 0],
            [0, 1, 0, m],
            [0, 0, 0, 1]
        ]
        t65 = [
            [cos(q[5]), 0, -sin(q[5]), 0],
            [sin(q[5]), 0, cos(q[5]), 0],
            [0, -1, 0, 0],
            [0, 0, 0, 1]
        ]
        t76 = [
            [cos(q[6]), -sin(q[6]), 0, 0],
            [sin(q[6]), cos(q[6]), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ]

        t20 = np.dot(t10, t21)
        t30 = np.dot(t20, t32)
        t40 = np.dot(t30, t43)
        t50 = np.dot(t40, t54)
        t60 = np.dot(t50, t65)
        t70 = np.dot(t60, t76)

        # forward kinamatics
        pos7 = [0, 0, n, 1]
        pos0 = np.dot(t70, pos7)

        # calculate end difference
        diff_x = pos0[0] - x
        diff_y = pos0[1] - y
        diff_z = pos0[2] + k - z

        diff_pos = [diff_x, diff_y, diff_z]

        ori1 = np.cross(
            np.dot(R, [1, 0, 0]),
            np.dot(t70, [1, 0, 0, 0])[:3]
        )
        ori2 = np.cross(
            np.dot(R, [0, 1, 0]),
            np.dot(t70, [0, 1, 0, 0])[:3]
        )
        ori3 = np.cross(
            np.dot(R, [0, 0, 1]),
            np.dot(t70, [0, 0, 1, 0])[:3]
        )
        diff_ori = 1/2*(ori1 + ori2 + ori3)

        diff_end = np.concatenate((diff_pos, diff_ori))

        # calculate jacobian
        vec1 = [0, 0, 1, 0]
        vec2 = [0, 0, 0, 1]

        z0 = [0, 0, 1]
        z1 = np.dot(t10, vec1)[:3]
        z2 = np.dot(t20, vec1)[:3]
        z3 = np.dot(t30, vec1)[:3]
        z4 = np.dot(t40, vec1)[:3]
        z5 = np.dot(t50, vec1)[:3]
        z6 = np.dot(t60, vec1)[:3]

        p0 = [0, 0, 0]
        p1 = np.dot(t10, vec2)[:3]
        p2 = np.dot(t20, vec2)[:3]
        p3 = np.dot(t30, vec2)[:3]
        p4 = np.dot(t40, vec2)[:3]
        p5 = np.dot(t50, vec2)[:3]
        p6 = np.dot(t60, vec2)[:3]
        pe = np.dot(t70, vec2)[:3]

        jacobian = np.transpose(
            [
                np.concatenate((np.cross(z0, pe-p0), z0)),
                np.concatenate((np.cross(z1, pe-p1), z1)),
                np.concatenate((np.cross(z2, pe-p2), z2)),
                np.concatenate((np.cross(z3, pe-p3), z3)),
                np.concatenate((np.cross(z4, pe-p4), z4)),
                np.concatenate((np.cross(z5, pe-p5), z5)),
                np.concatenate((np.cross(z6, pe-p6), z6)),
            ]
        )

        # calculate inverse jacobian
        jacobian_inv = np.linalg.pinv(jacobian)

        # calculate config difference
        diff_config = np.dot(jacobian_inv, diff_end)

        # calculate end error
        error_end = np.linalg.norm(diff_end)
        
        # update config
        q = q - diff_config

    return q

# -*- coding: utf-8 -*-

""" Forward and inverse kinematics computations.
"""

__author__ = 'Eric Pascual'

import math


class ArmKinematics(object):
    """ FK and IK for a planar serial arm, composed of :

        - a rotating base
        - a 1 DOF shoulder
        - a 1 DOF elbow
        - a 1 DOF wrist
        - a gripper
    """
    def __init__(self, segment_lengths):
        """
        :param tuple segment_lengths: a tuple of 3 floats providing the arm segment lengths (arm, forearm, gripper)
        """
        self.l1, self.l2, self.l3 = segment_lengths

    def ik(self, x, y, z, pitch):
        """ Inverse kinematics computation.

        :param float x, y, z: coordinates of the gripper (same units as lengths)
        :param float pitch: pitch angle of the gripper main axis
        :return: angles of the 4 joints (in radians)
        :rtype: tuple of float
        :raise: ValueError if the goal position is out of reach of if no solution can be found
        """
        dist = math.sqrt(x * x + y * y + z * z)
        if dist > self.l1 + self.l2 + self.l3:
            raise ValueError('out of reach')

        sin_pitch = math.sin(pitch)
        cos_pitch = math.cos(pitch)
        l1_2 = self.l1 * self.l1
        l2_2 = self.l2 * self.l2
        l3_2 = self.l3 * self.l3

        alpha = math.atan2(x, z)
        z_1 = math.sqrt(x * x + z * z)
        l_2 = z_1 * z_1 + y * y
        # l = math.sqrt(l_2)
        _1 = y - self.l3 * sin_pitch
        _2 = z_1 - self.l3 * cos_pitch
        l_3_2 = _1 * _1 + _2 * _2
        l_3 = math.sqrt(l_3_2)
        beta_1 = math.atan2(y - self.l3 * sin_pitch, z_1 - self.l3 * cos_pitch)
        beta_2 = math.atan2(y, z_1)
        beta_3 = math.acos((l1_2 + l_3_2 - l2_2) / 2 / self.l1 / l_3)
        beta = beta_1 + beta_3
        gamma = math.acos((l1_2 + l2_2 - l_3_2) / 2 / self.l1 / self.l2)
        delta_3 = math.pi - beta_3 - gamma
        delta_1 = math.acos(max(-1, min(1, (l_3_2 + l3_2 - l_2) / 2 / l_3 / self.l3)))
        delta = delta_1 + delta_3 if beta_1 >= beta_2 else 2 * math.pi - delta_1 + delta_3

        return alpha, beta, gamma, delta

    def fk(self, alpha, beta, gamma, delta, pitch):
        """ Direct kinematics computation.

        :param float x, y, z: coordinates of the gripper (same units as lengths)
        :param float pitch: pitch angle of the gripper main axis
        :return: angles of the 4 joints (in radians)
        :rtype: tuple of float
        :raise: ValueError if the goal position is out of reach of if no solution can be found
        """
        l1_2 = self.l1 * self.l1
        l2_2 = self.l2 * self.l2
        l3_2 = self.l3 * self.l3

        l_3_2 = l1_2 + l2_2 - 2 * self.l1 * self.l2 * math.cos(gamma)
        l_3 = math.sqrt(l_3_2)
        beta_3 = math.acos(round((l1_2 + l_3_2 - l2_2) / 2 / self.l1 / l_3, 3))
        beta_1 = beta - beta_3
        delta_3 = math.pi - beta_3 - gamma
        l_2 = l_3_2 + l3_2 - 2 * l_3 * self.l3 * math.cos(delta - delta_3)
        l = math.sqrt(l_2)
        z_1 = l_3 * math.cos(beta_1) + self.l3 * math.cos(pitch)
        beta_2 = math.acos(z_1 / l)
        y = l * math.sin(beta_2)
        z = z_1 * math.cos(alpha)
        x = z_1 * math.sin(alpha)

        return x, y, z

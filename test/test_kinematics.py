#!/usr/bin/env python
#  -*- coding: utf-8 -*-

__author__ = 'Eric Pascual'

import unittest
import math

from dynamixel.kinematics import ArmKinematics


class IKCase(unittest.TestCase):
    def setUp(self):
        self.geom = ArmKinematics((104.4, 104.4, 99.0))

    def test_01(self):
        alpha, beta, gamma, delta = self.geom.ik(
            x=0,
            y=0,
            z=self.geom.l1 + self.geom.l2 + self.geom.l3,
            pitch=0
        )
        self.assertEqual(alpha, 0)
        self.assertEqual(beta, 0)
        self.assertAlmostEqual(gamma, math.pi, 4)
        self.assertAlmostEqual(delta, math.pi, 4)

    def test_02(self):
        alpha, beta, gamma, delta = self.geom.ik(
            x=0,
            y=self.geom.l1 + self.geom.l2 + self.geom.l3,
            z=0,
            pitch=math.pi / 2
        )
        self.assertEqual(alpha, 0)
        self.assertEqual(beta, math.pi / 2)
        self.assertAlmostEqual(gamma, math.pi, 4)
        self.assertAlmostEqual(delta, math.pi, 4)

    def test_03(self):
        alpha, beta, gamma, delta = self.geom.ik(
            x=0,
            y=(self.geom.l1 + self.geom.l2 + self.geom.l3) * math.sin(math.pi / 4),
            z=(self.geom.l1 + self.geom.l2 + self.geom.l3) * math.cos(math.pi / 4),
            pitch=math.pi / 4
        )
        self.assertEqual(alpha, 0)
        self.assertAlmostEqual(beta, math.pi / 4, 4)
        self.assertAlmostEqual(gamma, math.pi, 4)
        self.assertAlmostEqual(delta, math.pi, 4)

    def test_04(self):
        alpha, beta, gamma, delta = self.geom.ik(
            x=0,
            y=0,
            z=self.geom.l1 * math.cos(math.pi / 4) + self.geom.l2 * math.cos(math.pi / 4) + self.geom.l3,
            pitch=0
        )
        self.assertEqual(alpha, 0)
        self.assertAlmostEqual(beta, math.pi / 4, 4)
        self.assertAlmostEqual(gamma, math.pi / 2, 4)
        self.assertAlmostEqual(delta, math.pi * 5 / 4, 4)

    def test_05(self):
        alpha, beta, gamma, delta = self.geom.ik(
            x=0,
            y=-self.geom.l3,
            z=self.geom.l1 * math.cos(math.pi / 4) + self.geom.l2 * math.cos(math.pi / 4),
            pitch=-math.pi / 2
        )
        self.assertEqual(alpha, 0)
        self.assertAlmostEqual(beta, math.pi / 4, 4)
        self.assertAlmostEqual(gamma, math.pi / 2, 4)
        self.assertAlmostEqual(delta, math.pi * 3 / 4, 4)

    def test_06_out_of_reach(self):
        with self.assertRaises(ValueError):
            self.geom.ik(
                x=0,
                y=0,
                z=self.geom.l1 + self.geom.l2 + self.geom.l3 + 10,
                pitch=0
            )

if __name__ == '__main__':
    unittest.main()

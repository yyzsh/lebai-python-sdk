#!/usr/bin/python
# -*- coding: utf-8 -*-
import logging
import unittest

from lebai import LebaiRobot, LebaiScene
# n)umber
from lebai.lebai_http_service import LebaiHttpService
from lebai.type import CartesianPose, JointPose, PVAT


class Test(unittest.TestCase):
    """Test http_api_test.py"""
    ip = "192.168.3.227"

    @classmethod
    def setUpClass(cls):
        LOG_FORMAT = "%(asctime)s - %(levelname)s - %(message)s"
        logging.basicConfig(level=logging.INFO, format=LOG_FORMAT)
        pass

    @classmethod
    def tearDownClass(cls):
        pass

    # http

    def setUp(self):
        self.robot = LebaiRobot(self.ip, True)

    def tearDown(self):
        pass

    def test_movej(self):
        self.robot.movej(
            JointPose(0, -0.7853981633974483, 1.5707963267948966, -0.7853981633974483, 1.5707963267948966, 0), 1.23,
            1.23)
        pass

    def test_movec(self):
        self.robot.movec(JointPose(0.2, 0.5, 0.4, 0, 0, 1.57), CartesianPose(0.1, 0.2, 0.2, 0.3, 0.1, 0.2), 0, 1, 0.2,
                         0)
        pass

    def test_scene_run(self):
        scene = LebaiScene(self.ip, 10003)
        scene.run(loop=1, timeout=3)
        pass

    def test_get_signal(self):
        self.robot.set_fan(0)
        pass


if __name__ == '__main__':
    unittest.main(verbosity=1)

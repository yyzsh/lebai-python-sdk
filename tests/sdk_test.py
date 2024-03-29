#!/usr/bin/python
# -*- coding: utf-8 -*-
import logging
import unittest
import requests
from lebai import LebaiRobot, LebaiScene
from lebai import *
# n)umber
from lebai.lebai_http_service import LebaiHttpService
from lebai.type import CartesianPose, JointPose, PVAT


class Test(unittest.TestCase):
    """Test http_api_test.py"""
    ip = "192.168.3.218"

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
    def test_movec(self):
        self.robot.move_pvat(JointPose(0.2, 0.5, 0.4, 0, 0, 1.57), [0.1, 0.2, 0.2, 0.3, 0.1, 0.2], 0, 1, 0.2,
                         0)
        pass

    def test_scene_run(self):
        scene = LebaiScene(self.ip, 10003)
        scene.run(loop=1, timeout=3)
        pass

    def test_get_signal(self):
        self.robot.get_ao_mode(0)
        pass
    def test_get_signal2(self):
        requests.post()
        pass

    
    def test_get_dh_params(self):
        r = self.robot.get_dh_params()
        for s in r:
            print(s.a)
            print(s.alpha)
            print(s.d)
            print(s.theta)
        pass


if __name__ == '__main__':
    unittest.main(verbosity=1)

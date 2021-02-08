#!/usr/bin/python3
# -*- coding: utf-8 -*-

import logging

from lebai import LebaiRobot, CartesianPose, JointPose

def run():
    rb = LebaiRobot("192.168.3.218")
    print(f"get_joint_temp(1) = {rb.get_joint_temp(1):.3f}")

    rb.movej(JointPose(0, 0, 0, 0, 0, 0), 0, 0, 1, 0)
    rb.movej(CartesianPose(-0.54, -0.2, 0.117, 0, 0, 0), 0, 0, 1, 0)

if __name__ == '__main__':
    logging.basicConfig()
    run()

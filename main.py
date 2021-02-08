#!/usr/bin/python3
# -*- coding: utf-8 -*-

import logging

from lebai import LebaiRobot

def run():
    rb = LebaiRobot("192.168.3.218")
    print(f"get_joint_temp(1) = {rb.get_joint_temp(1):.3f}")

if __name__ == '__main__':
    logging.basicConfig()
    run()

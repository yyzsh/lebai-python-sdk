#!/usr/bin/python3
# -*- coding: utf-8 -*-

import logging
import sys

from lebai import LebaiRobot


def run():
    ip = sys.argv[1] if sys.argv[1] else "192.168.3.218"
    robot = LebaiRobot(ip, True)
    data = robot.get_tasks(0, 10)
    print(data)
    print(robot.get_task(data['records'][0]['id']))
    code = ('local p = kinematics_inverse({12,12,1,2,1,1})'
            'for k,v in ipairs(p) do'
            'print(k,v)'
            'end'
            'print(p["ok"])'
            )
    print(robot.execute_lua_code('xxx', 1, 0, code))


if __name__ == '__main__':
    LOG_FORMAT = "%(asctime)s - %(levelname)s - %(message)s"
    logging.basicConfig(level=logging.INFO, format=LOG_FORMAT)
    run()

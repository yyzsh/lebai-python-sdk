#!/usr/bin/python3
# -*- coding: utf-8 -*-

import logging
import asyncio
import math

from lebai import LebaiRobot, CartesianPose, JointPose
from grpc.aio import AioRpcError

async def run():
    rb = LebaiRobot("192.168.3.218")
    print(f"get_joint_temp(1) = {(await rb.get_joint_temp(1)):.3f}")

    await rb.movej(JointPose(0, 0, 0, 0, 0, 0), 0, 0, 1, 0)
    print(await rb.get_actual_joint_positions())
    try:
        p = CartesianPose(-0.54, -0.2, 0.117, 0, math.pi / 2, math.pi / 2)
        await rb.movej(p, 0, 0, 1, 0)
    except AioRpcError as e:
        print(e.code(), e.details())
    print(await rb.get_actual_joint_positions())
    base = await rb.get_actual_tcp_pose()
    print('base=', base, type(base))
    p1 = CartesianPose(0.1, 0, 0, 0, 0, 0, base=base)
    print(p1, p1.base)
    await rb.movej(p1, 0, 0, 1, 0)
    print(await rb.get_actual_tcp_pose())

if __name__ == '__main__':
    logging.basicConfig()
    asyncio.run(run())

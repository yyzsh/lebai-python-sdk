#!/usr/bin/python3
# -*- coding: utf-8 -*-

import logging
import asyncio
import math

from lebai import LebaiRobot, CartesianPose, JointPose
from grpc.aio import AioRpcError

async def get_v(rb, n=10, sec=0.01):
    for i in range(0, n):
        logging.info(await rb.get_actual_joint_speeds())
        await asyncio.sleep(sec)

async def run1():
    rb = LebaiRobot("192.168.3.218")
    await rb.start_sys()
    logging.info(await rb.get_robot_mode())

    await rb.set_velocity_factor(60)
    logging.info(await rb.get_velocity_factor())

    logging.info(await rb.get_actual_joint_torques())
    logging.info(await rb.get_target_joint_torques())
    logging.info(await rb.get_joint_temperatures())

    p1 = JointPose(0, -0.5, math.pi/6, 0, 0, 0)
    logging.info(p1)
    p1_c = await rb.kinematics_forward(p1)
    logging.info(p1_c)
    logging.info(await rb.get_actual_joint_positions())
    p1_j = await rb.kinematics_inverse(p1_c)
    logging.info(p1_j)
    logging.info(await rb.kinematics_forward(p1_j))

    logging.info(await rb.get_robot_io_data())
    logging.info(await rb.get_robot_data())

    g = await rb.get_gravity()
    await rb.set_gravity(g)
    assert(await rb.get_gravity() == g)
    await rb.set_gravity(g[0], g[1], g[2])
    assert(await rb.get_gravity() == g)

    p = ((0.1, 0.2, 0.3), 0.12)
    await rb.set_payload_mass(p[1])
    assert(await rb.get_payload_mass() == p[1])
    await rb.set_payload_cog(p[0])
    assert(await rb.get_payload_cog() == p[0])
    await rb.set_payload(p)
    assert(await rb.get_payload() == p)
    await rb.set_payload((0, -9.8, 0), 0)
    assert(await rb.get_payload() == ((0, -9.8, 0), 0))
    await rb.set_payload(x=0.1, y=-0.1, z=-9.8, mass=0.3)
    assert(await rb.get_payload() == ((0.1, -0.1, -9.8), 0.3))
    await rb.set_payload(0, 0, -9.8, 0)
    assert(await rb.get_payload() == ((0, 0, -9.8), 0))

    t = await rb.get_tcp()
    await rb.set_tcp(t)
    assert(await rb.get_tcp() == t)
    t = CartesianPose(0, 0, 0, 0, 0, 0)
    await rb.set_tcp(t)
    assert(await rb.get_tcp() == t)

    logging.info(f"get_joint_temp(1) = {(await rb.get_joint_temp(1)):.3f}")

    await rb.disable_joint_limits()

    await rb.movej(JointPose(0, -0.5, math.pi/6, 0, 0, 0), 0, 0, 1, 0)
    logging.info(await rb.get_actual_joint_positions())

    try:
        p = CartesianPose(-0.54, -0.2, 0.117, 0, math.pi / 2, math.pi / 2)
        await rb.movej(p, 0, 0, 1, 0)
    except AioRpcError as e:
        logging.info(e.code(), e.details())

    logging.info(await rb.get_actual_joint_positions())

    base = await rb.get_actual_tcp_pose()
    # logging.info('base=', base, type(base))

    p1 = CartesianPose(0, 0.1, 0, 0, 0, 0, base=base)
    # logging.info(p1, p1.base)
    m1 = asyncio.create_task(rb.movej(p1, 0, 0, 1, 0))
    m2 = asyncio.create_task(get_v(rb, 10, 0.1))
    # logging.info(await rb.get_actual_tcp_pose())
    await m1

    p2 = CartesianPose(0.1, 0.2, 0, 0, 0, 0, base=base)
    await rb.movel(p2, 0, 0, 2, 0)
    logging.info(await rb.get_actual_tcp_pose())

    await rb.movec(CartesianPose(0.1, 0, 0, 0, 0, 0), p1, rad=-math.pi/3, t=5)
    await m2

    await rb.stop_sys()

    await rb.enable_joint_limits()

async def run(x):
    for i in range(0, x):
        await run1()

if __name__ == '__main__':
    LOG_FORMAT = "%(asctime)s - %(levelname)s - %(message)s"
    logging.basicConfig(level=logging.INFO, format=LOG_FORMAT)
    asyncio.run(run(1))

#!/usr/bin/python3
# -*- coding: utf-8 -*-

import logging
import math

from lebai import LebaiRobot, CartesianPose, JointPose

def run():
    rb = LebaiRobot("192.168.3.218")
    rb.start_sys()
    logging.info(rb.get_robot_mode())

    rb.set_velocity_factor(60)
    logging.info(rb.get_velocity_factor())
    logging.info(rb.get_di(1))
    rb.set_do(1, 1)

    logging.info(rb.get_actual_joint_torques())
    logging.info(rb.get_target_joint_torques())
    logging.info(rb.get_joint_temperatures())

    p1 = JointPose(0, -0.5, math.pi/6, 0, 0, 0)
    logging.info(p1)
    p1_c = rb.kinematics_forward(p1)
    logging.info(p1_c)
    logging.info(rb.get_actual_joint_positions())
    p1_j = rb.kinematics_inverse(p1_c)
    logging.info(p1_j)
    logging.info(rb.kinematics_forward(p1_j))

    logging.info(rb.get_robot_io_data())
    logging.info(rb.get_robot_data())

    g = rb.get_gravity()
    rb.set_gravity(g)
    assert(rb.get_gravity() == g)
    rb.set_gravity(g[0], g[1], g[2])
    assert(rb.get_gravity() == g)

    p = ((0.1, 0.2, 0.3), 0.12)
    rb.set_payload_mass(p[1])
    logging.info(rb.get_payload_mass())
    assert(rb.get_payload_mass() == p[1])
    rb.set_payload_cog(p[0])
    assert(rb.get_payload_cog() == p[0])
    rb.set_payload(p)
    assert(rb.get_payload() == p)
    rb.set_payload((0, -9.8, 0), 0)
    assert(rb.get_payload() == ((0, -9.8, 0), 0))
    rb.set_payload(x=0.1, y=-0.1, z=-9.8, mass=0.3)
    assert(rb.get_payload() == ((0.1, -0.1, -9.8), 0.3))
    rb.set_payload(0, 0, -9.8, 0)
    assert(rb.get_payload() == ((0, 0, -9.8), 0))

    t = rb.get_tcp()
    rb.set_tcp(t)
    assert(rb.get_tcp() == t)
    t = CartesianPose(0, 0, 0, 0, 0, 0)
    rb.set_tcp(t)
    assert(rb.get_tcp() == t)

    logging.info(f"get_joint_temp(1) = {(rb.get_joint_temp(1)):.3f}")

    rb.disable_joint_limits()

    rb.movej(JointPose(0, -0.5, math.pi/6, 0, 0, 0), 0, 0, 1, 0)
    logging.info(rb.get_actual_joint_positions())

    try:
        p = CartesianPose(-0.54, -0.2, 0.117, 0, math.pi / 2, math.pi / 2)
        rb.movej(p, 0, 0, 1, 0)
    except Exception as e:
        logging.info(e.code(), e.details())

    logging.info(rb.get_actual_joint_positions())

    base = rb.get_actual_tcp_pose()
    # logging.info('base=', base, type(base))

    p1 = CartesianPose(0, 0.1, 0, 0, 0, 0, base=base)
    # logging.info(p1, p1.base)
    rb.movej(p1, 0, 0, 1, 0)
    # logging.info(rb.get_actual_tcp_pose())

    p2 = CartesianPose(0.1, 0.2, 0, 0, 0, 0, base=base)
    rb.movel(p2, 0, 0, 2, 0)
    logging.info(rb.get_actual_tcp_pose())

    rb.movec(CartesianPose(0.1, 0, 0, 0, 0, 0), p1, rad=-math.pi/3, t=5)

    rb.stop_sys()

    rb.enable_joint_limits()

if __name__ == '__main__':
    LOG_FORMAT = "%(asctime)s - %(levelname)s - %(message)s"
    logging.basicConfig(level=logging.INFO, format=LOG_FORMAT)
    run()

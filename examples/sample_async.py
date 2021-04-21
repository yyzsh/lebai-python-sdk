import asyncio
import math

from lebai import LebaiRobot, CartesianPose, JointPose

async def run():
    rb = LebaiRobot("192.168.3.218")

    await rb.start_sys()

    await rb.movej(JointPose(0, -1.2, math.pi/6, 0, math.pi/4, 0), 0, 0, 1, 0)

    base = await rb.get_actual_tcp_pose()
    p2 = CartesianPose(0.1, 0, 0, 0, 0, 0, base=base)
    await rb.movel(p2, 0, 0, 1, 0)

    await rb.stop_sys()

if __name__ == '__main__':
    asyncio.run(run())
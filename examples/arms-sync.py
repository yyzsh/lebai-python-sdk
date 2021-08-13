import math

from lebai import LebaiRobot, CartesianPose, JointPose


def run():
    rb1 = LebaiRobot("192.168.3.52")
    rb2 = LebaiRobot("192.168.3.80")

    rb2.move_pvts(rb1.record_pvat())


if __name__ == '__main__':
    run()
from lebai import LebaiRobot

if __name__ == '__main__':
    l1 = LebaiRobot('127.0.0.1', port=3030)
    print(l1.get_robot_data())
    l1.set_velocity_factor(20)
    assert(l1.get_velocity_factor() == 20)

    l2 = LebaiRobot('127.0.0.1', port=3030)

    l2.set_gravity(1, 2, 3)
    print(l2.get_gravity())

    print(l2.get_tcp())

    l2.set_payload(1, 2, 3, 4)
    print(l2.get_payload())

    l1.close()

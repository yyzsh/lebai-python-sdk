from .ws import JsonRpcClient
from .type import *


class LebaiRobotV3:
    def __init__(self, ip, sync=True, port=3031, debug=False):
        self.ip = ip
        self.port = port
        self._sync = sync

        self.ws = JsonRpcClient(ip, port, debug=debug)

    def send(self, cmd, data=None, sync=None):
        if sync is None:
            sync = self._sync
        # if sync:
        #     self.ws.send('sync', sync=True)
        return self.ws.send(cmd, data, sync=sync)

    def start_sys(self):
        return self.send('start_sys')

    def get_robot_mode(self):
        return RobotState(self.send('get_robot_mode'))

    def is_connected(self):
        return self.get_robot_mode() != RobotState.DISCONNECTED

    def set_velocity_factor(self, val):
        return self.send('set_velocity_factor', [val])

    def get_velocity_factor(self):
        return self.send('get_velocity_factor')

    def get_di(self, pin, dev):
        return self.send('get_di', [pin, dev])

    def set_do(self, pin, val, dev):
        return self.send('set_do', [pin, val, dev])

    def get_actual_joint_torques(self):
        return self.send('get_actual_joint_torques')

    def get_target_joint_torques(self):
        return self.send('get_target_joint_torques')

    def get_actual_joint_positions(self):
        return self.send('get_actual_joint_positions')

    def get_target_joint_positions(self):
        return self.send('get_target_joint_positions')

    def get_joint_temperatures(self):
        return self.send('get_joint_temperatures')

    def kinematics_forward(self, p):
        j = p
        if getattr(p, 'is_joint'):
            j = p.pos
        return CartesianPose(self.send('kinematics_forward', [j]))

    def kinematics_inverse(self, p):
        c = p
        if hasattr(p, 'pos'):
            c = p.pos
        return self.send('kinematics_inverse', [c])

    def get_robot_io_data(self):
        id = self.send('sub_robot_io_data')
        yield self.ws.recv()
        self.send('unsub_robot_io_data', [id])

    def get_robot_data(self):
        return RobotData(None, self.send('get_robot_data'))

    def set_gravity(self, x = 0, y = 0, z = -9.8) -> None:
        if type(x) is tuple or type(x) is list:
            y = x[1]
            z = x[2]
            x = x[0]
        return self.send('set_gravity', [x, y, z])

    def get_gravity(self):
        return self.send('get_gravity')

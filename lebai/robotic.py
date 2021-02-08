import grpc
import logging
import asyncio

from .pb2 import robot_controller_pb2_grpc, robot_controller_pb2

class LebaiRobot(object):
    '''
    :param ip: 机器人设备 IP
    
    :returns: 返回一个乐白机器人控制实例。

    可以为同一台设备创建多个实例，但同一台机器人同时只能执行有限的指令。
    '''
    def __init__(self, ip):
        self.rcc = grpc.insecure_channel(f'{ip}:5181')
        self.rcs = robot_controller_pb2_grpc.RobotControllerStub(self.rcc)

    def get_joint_temp(self, joint):
        '''获取关节温度

        :param joint: 关节序号

        :returns: 关节当前温度 (℃)
        '''
        response = self.rcs.GetJointTemp(robot_controller_pb2.IntRequest(index=joint))
        return response.degree

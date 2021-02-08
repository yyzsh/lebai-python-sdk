import grpc
import logging
import asyncio

from .pb2 import robot_controller_pb2_grpc, robot_controller_pb2

class LebaiRobot(object):
    def __init__(self, ip):
        self.rcc = grpc.insecure_channel(f'{ip}:5181')
        self.rcs = robot_controller_pb2_grpc.RobotControllerStub(self.rcc)
    
    def __del__(self):
        del self.rcs
        del self.rcc
    
    def get_joint_temp(self, joint):
        response = self.rcs.GetJointTemp(robot_controller_pb2.IntRequest(index=joint))
        return response.degree

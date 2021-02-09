import grpc
import logging
import asyncio
from enum import Enum, unique

from .pb2 import robot_controller_pb2_grpc
from .pb2 import robot_controller_pb2 as rc
from .pb2 import messages_pb2 as msg
from google.protobuf.empty_pb2 import Empty

class RobotState(Enum):
    '''机器人状态
    '''
    DISCONNECTED = 0  # 已断开连接
    ESTOP = 1         # 急停停止状态
    BOOTING = 2       # 启动中
    ROBOT_OFF = 3     # 电源关闭
    ROBOT_ON = 4      # 电源开启
    IDLE = 5          # 空闲中
    PAUSED = 6     # 暂停中
    RUNNING = 7       # 机器人运动运行中
    UPDATING = 8      # 更新固件中
    STARTING = 9      # 启动中
    STOPPING = 10     # 停止中
    TEACHING = 11     # 示教中
    STOP = 12         # 普通停止
    FINETUNING = 13   # 微调中

class CartesianPose:
    '''笛卡尔坐标描述的位姿
    '''
    def __init__(self, x, y, z, rz, ry, rx, base=None):
        self.pos = (x, y, z, rz, ry, rx)
        self.is_joint = False
        self.base = getattr(base, 'pos', base) if base is not None else None

    def __str__(self):
        if self.base is None:
            return str(self.pos)
        else:
            return f'({self.pos[0]}, {self.pos[1]}, {self.pos[2]}, {self.pos[3]}, {self.pos[4]}, {self.pos[5]}, base={self.base})'

class JointPose:
    '''关节旋转角度描述的机器人姿态
    '''
    def __init__(self, *j):
        self.pos = j
        self.is_joint = True

    def __str__(self):
        return str(self.pos)

class LebaiRobot:
    '''
    :param ip: 机器人设备 IP

    :returns: 返回一个乐白机器人控制实例

    可以为同一台设备创建多个实例，但同一台机器人同时只能执行有限的指令。
    '''
    def __init__(self, ip):
        self.rcc = grpc.aio.insecure_channel(f'{ip}:5181')
        self.rcs = robot_controller_pb2_grpc.RobotControllerStub(self.rcc)

    async def movej(self, pos, a, v, t, r):
        '''线性移动（关节空间）

        :param pos:
            `JointPose` 关节位置
            `CartesianPose` 末端位置（将通过运动学反解转为关节位置）
        :param a: 主轴的关节加速度 (rad/s2)
        :param v: 主轴的关节速度 (rad/s)
        :param t: 运动时间 (s)
        :param r: 交融半径 (m)
        '''
        req = rc.MoveJRequest(
            joint_pose_to = list(pos.pos),
            pose_is_joint_angle=getattr(pos, 'is_joint', False),
            acceleration=a,
            velocity=v,
            time=t,
            blend_radius=r
        )
        if getattr(pos, 'base', None):
            req.pose_base.position.x = pos.base[0]
            req.pose_base.position.y = pos.base[1]
            req.pose_base.position.z = pos.base[2]
            req.pose_base.rotation.r = pos.base[3]
            req.pose_base.rotation.p = pos.base[4]
            req.pose_base.rotation.y = pos.base[5]
        res = await self.rcs.MoveJ(req)
        return res

    async def get_actual_joint_positions(self):
        '''获取实际关节位置

        :returns: 
        '''
        res = await self.rcs.GetActualJointPositions(Empty())
        return JointPose(*res.joints)

    async def get_actual_tcp_pose(self):
        '''获取实际空间位置

        :returns: 
        '''
        res = await self.rcs.GetActualTcpPose(Empty())
        return CartesianPose(*res.vector)

    async def get_joint_temp(self, joint):
        '''获取关节温度

        :param joint: 关节序号

        :returns: 关节当前温度 (℃)
        '''
        res = await self.rcs.GetJointTemp(rc.IntRequest(index=joint))
        return res.degree

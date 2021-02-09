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
    PAUSED = 6        # 暂停中
    RUNNING = 7       # 机器人运动运行中
    UPDATING = 8      # 更新固件中
    STARTING = 9      # 启动中
    STOPPING = 10     # 停止中
    TEACHING = 11     # 示教中
    STOP = 12         # 普通停止
    FINETUNING = 13   # 微调中

class CartesianPose:
    '''空间位置
    
    笛卡尔坐标描述的位姿
    '''
    def __init__(self, x=0, y=0, z=0, rz=0, ry=0, rx=0, base=None):
        if type(x) is msg.PR:
            self.pos = (x.position.x, x.position.y, x.position.z, x.rotation.r, x.rotation.p, x.rotation.y)
        elif type(x) is CartesianPose:
            self.pos = x.pos[:]
            base = x.base
            print(self.pos)
        else:
            self.pos = (x, y, z, rz, ry, rx)
        self.is_joint = False
        # 要求 base 是元组或列表而不是 CartesianPose 对象
        self.base = getattr(base, 'pos', base) if base is not None else None

    def __str__(self):
        if self.base is None:
            return 'CartesianPose' + str(self.pos)
        else:
            return f'CartesianPose({self.pos[0]}, {self.pos[1]}, {self.pos[2]}, {self.pos[3]}, {self.pos[4]}, {self.pos[5]}, base={self.base})'

    def __getattr__(self, key):
        pos_idx_arr = ['x', 'y', 'z', 'rz', 'ry', 'rx']
        if key in pos_idx_arr:
            idx = pos_idx_arr.index(key)
            return getattr(self, 'pos')[idx]
        else:
            return getattr(self, key)

    def __eq__(self, other):
        return self.pos == other.pos and self.base == other.base

    def to_PR(self):
        p = msg.Coordinate(x=self.pos[0], y=self.pos[1], z=self.pos[2])
        r = msg.Rotation(r=self.pos[3], p=self.pos[4], y=self.pos[5])
        return msg.PR(position=p,rotation=r)

class JointPose:
    '''关节位置
    
    关节旋转角度描述的机器人姿态
    '''
    def __init__(self, *j):
        self.pos = j
        self.is_joint = True

    def __str__(self):
        return 'CartesianPose' + str(self.pos)

class LebaiRobot:
    '''
    :param ip: 机器人设备 IP

    :returns: 返回一个乐白机器人控制实例
    '''
    def __init__(self, ip):
        self.rcc = grpc.aio.insecure_channel(f'{ip}:5181')
        self.rcs = robot_controller_pb2_grpc.RobotControllerStub(self.rcc)

    async def start_sys(self):
        pass

    async def stop_sys(self):
        pass

    async def powerdown(self):
        await self.rcs.PowerDown(Empty)
        return True

    async def stop(self):
        pass

    async def estop(self):
        pass

    async def teach_mode(self):
        await self.rcs.TeachMode(Empty)
        return True

    async def end_teach_mode(self):
        await self.rcs.EndTeachMode(Empty)
        return True

    async def wait(self):
        pass

    async def resume(self):
        pass

    async def pause(self):
        pass

    async def get_robot_mode(self):
        pass

    async def get_velocity_factor(self):
        res = await self.rcs.GetVelocityFactor(Empty())
        return res.value

    async def set_velocity_factor(self, factor):
        await self.rcs.SetVelocityFactor(rc.Factor(value=factor))
        return True

    async def set_gravity(self, x=0, y=0, z=-9.8):
        if type(x) is tuple or type(x) is list:
            y = x[1]
            z = x[2]
            x = x[0]
        await self.rcs.SetGravity(msg.Coordinate(x=x,y=y,z=z))
        return True

    async def get_gravity(self):
        res = await self.rcs.GetGravity(Empty())
        return (res.x, res.y, res.z)

    async def set_payload(self, x=0, y=0, z=-9.8, mass=0):
        if type(x) is tuple:
            if type(y) is not tuple and type(x[0]) is tuple:
                y = x[1]
                x = x[0]
            mass = y
            z = x[2]
            y = x[1]
            x = x[0]
        await self.rcs.SetPayload(msg.Payload(mass=mass, cog=msg.Coordinate(x=x,y=y,z=z)))
        return True

    async def get_payload(self):
        res = await self.rcs.GetPayload(Empty())
        return ((res.cog.x, res.cog.y, res.cog.z), res.mass)

    async def set_payload_mass(self, mass):
        await self.rcs.SetPayloadMass(msg.PayloadMass(mass=mass))
        return True

    async def get_payload_mass(self):
        res = await self.rcs.GetPayloadMass(Empty())
        return res.mass

    async def set_payload_cog(self, x=0, y=0, z=0):
        if type(x) is tuple or type(x) is list:
            y = x[1]
            z = x[2]
            x = x[0]
        await self.rcs.SetPayloadCog(msg.PayloadCog(cog=msg.Coordinate(x=x,y=y,z=z)))
        return True

    async def get_payload_cog(self):
        res = await self.rcs.GetPayloadCog(Empty())
        return (res.cog.x, res.cog.y, res.cog.z)

    async def set_tcp(self, x=0, y=0, z=0, rz=0, ry=0, rx=0):
        tcp = CartesianPose(x, y, z, rz, ry, rx)
        await self.rcs.SetTcp(tcp.to_PR())
        return True

    async def get_tcp(self):
        res = await self.rcs.GetTcp(Empty())
        return CartesianPose(res)

    async def get_claw_aio(self):
        pass

    async def set_claw_aio(self):
        pass

    async def set_claw(self):
        pass

    async def movej(self, p, a=0, v=0, t=0, r=0):
        '''线性移动（关节空间）

        :param p:
            - `JointPose` 关节位置
            - `CartesianPose` 空间位置（将通过运动学反解转为关节位置）
        :param a: 主轴的关节加速度 (rad/s\ :sup:`2`)
        :param v: 主轴的关节速度 (rad/s)
        :param t: 运动时间 (s)
        :param r: 交融半径 (m)
        '''
        req = rc.MoveJRequest(
            joint_pose_to = list(p.pos),
            pose_is_joint_angle=getattr(p, 'is_joint', False),
            acceleration=a,
            velocity=v,
            time=t,
            blend_radius=r
        )
        if getattr(p, 'base', None) is not None:
            req.pose_base.position.x = p.base[0]
            req.pose_base.position.y = p.base[1]
            req.pose_base.position.z = p.base[2]
            req.pose_base.rotation.r = p.base[3]
            req.pose_base.rotation.p = p.base[4]
            req.pose_base.rotation.y = p.base[5]
        res = await self.rcs.MoveJ(req)
        return res

    async def movej_until(self):
        pass

    async def movej_until_rt(self):
        pass

    async def movel(self):
        pass

    async def movel_until(self):
        pass

    async def movel_until_rt(self):
        pass

    async def movec(self):
        pass

    async def movec_until(self):
        pass

    async def movec_until_rt(self):
        pass

    async def stop_move(self):
        pass

    async def kinematics_forward(self):
        pass

    async def kinematics_inverse(self):
        pass

    async def pose_times(self):
        pass

    async def pose_inverse(self):
        pass

    async def get_actual_joint_positions(self):
        '''获取实际关节位置

        :returns: `JointPose` 关节位置
        '''
        res = await self.rcs.GetActualJointPositions(Empty())
        return JointPose(*res.joints)

    async def get_target_joint_positions(self):
        pass

    async def get_actual_joint_speeds(self):
        pass

    async def get_target_joint_speeds(self):
        pass

    async def get_joint_torques(self):
        pass

    async def get_actual_joint_torques(self):
        pass

    async def get_target_joint_torques(self):
        pass

    async def get_joint_temperatures(self):
        pass

    async def get_joint_temp(self, joint):
        '''获取关节温度

        :param joint: 关节序号

        :returns: 关节当前温度 (℃)
        '''
        res = await self.rcs.GetJointTemp(rc.IntRequest(index=joint))
        return res.degree

    async def get_actual_tcp_pose(self):
        '''获取实际空间位置

        :returns: `CartesianPose` 空间位置
        '''
        res = await self.rcs.GetActualTcpPose(Empty())
        return CartesianPose(*res.vector)

    async def get_target_tcp_pose(self):
        pass

    async def get_robot_poses(self):
        pass

    async def get_robot_data(self):
        pass

    async def write_extra_servo_param(self):
        pass

    async def read_extra_servo_param(self):
        pass

    async def reset_extra_servo_param(self):
        pass

    async def write_extra_servo_params(self):
        pass

    async def read_extra_servo_params(self):
        pass

    async def reset_extra_servo_params(self):
        pass

    async def write_joint_backlash(self):
        pass

    async def read_joint_backlash(self):
        pass

    async def reset_joint_backlash(self):
        pass

    async def write_joint_backlashes(self):
        pass

    async def read_joint_backlashes(self):
        pass

    async def reset_joint_backlashes(self):
        pass

    async def read_enable_joint_backlash(self):
        pass

    async def read_enable_joint_backlashes(self):
        pass

    async def write_enable_joint_backlash(self):
        pass

    async def write_enable_joint_backlashes(self):
        pass

    async def write_joint_backlash_param(self):
        pass

    async def read_joint_backlash_param(self):
        pass

    async def reset_joint_backlash_param(self):
        pass

    async def write_joint_backlash_params(self):
        pass

    async def read_joint_backlash_params(self):
        pass

    async def reset_joint_backlash_params(self):
        pass

    async def set_dio(self):
        pass

    async def get_dio(self):
        pass

    async def set_aio(self):
        pass

    async def get_aio(self):
        pass

    async def set_tcp_dio(self):
        pass

    async def get_tcp_dio(self):
        pass

    async def set_tcp_aio(self):
        pass

    async def get_tcp_aio(self):
        pass

    async def set_external_do(self):
        pass

    async def set_external_dos(self):
        pass

    async def get_external_do(self):
        pass

    async def get_external_dos(self):
        pass

    async def get_external_di(self):
        pass

    async def get_external_dis(self):
        pass

    async def set_external_ao(self):
        pass

    async def set_external_aos(self):
        pass

    async def get_external_ao(self):
        pass

    async def get_external_aos(self):
        pass

    async def get_external_ai(self):
        pass

    async def get_external_ais(self):
        pass

    async def get_external_ios(self):
        pass

    async def set_ioboard_dio(self):
        pass

    async def get_ioboard_dio(self):
        pass

    async def set_ioboard_aio(self):
        pass

    async def get_ioboard_aio(self):
        pass

    async def set_led(self):
        pass

    async def set_voice(self):
        pass

    async def set_fan(self):
        pass

    async def signal_set(self):
        pass

    async def signal_get(self):
        pass

    async def signal_wait(self):
        pass

    async def signal_add(self):
        pass

    async def new_thread(self):
        pass

    async def new_coroutine(self):
        pass

    async def enable_joint_limits(self):
        pass

    async def disable_joint_limits(self):
        pass

    async def alert(self):
        pass

    async def confirm(self):
        pass

    async def input(self):
        pass

    async def select(self):
        pass

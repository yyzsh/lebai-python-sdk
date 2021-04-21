import grpc
import asyncio

from google.protobuf.empty_pb2 import Empty
from .pb2 import robot_controller_pb2_grpc
from .pb2 import private_controller_pb2_grpc

from .type import *

class LebaiRobotAsync:
    '''
    :param ip: 机器人设备 IP

    :returns: 返回一个乐白机器人控制实例
    '''
    def __init__(self, ip):
        self.rcc = grpc.aio.insecure_channel(f'{ip}:5181')
        self.rcs = robot_controller_pb2_grpc.RobotControllerStub(self.rcc)

        self.pcc = grpc.aio.insecure_channel(f'{ip}:5182')
        self.pcs = private_controller_pb2_grpc.RobotPrivateControllerStub(self.pcc)

    async def start_sys(self):
        await self.rcs.StartSys(Empty())

    async def stop_sys(self):
        await self.rcs.StopSys(Empty())

    async def powerdown(self):
        await self.rcs.PowerDown(Empty)

    async def stop(self):
        await self.rcs.Stop(Empty())

    async def estop(self):
        await self.rcs.EStop(Empty())

    async def teach_mode(self):
        await self.rcs.TeachMode(Empty)

    async def end_teach_mode(self):
        await self.rcs.EndTeachMode(Empty)

    async def resume(self):
        await self.rcs.Resume(Empty())

    async def pause(self):
        await self.rcs.Pause(Empty())

    async def get_robot_mode(self):
        res = await self.rcs.GetRobotMode(Empty())
        return RobotState(res.mode)

    async def get_velocity_factor(self):
        res = await self.rcs.GetVelocityFactor(Empty())
        return res.value

    async def set_velocity_factor(self, factor):
        await self.rcs.SetVelocityFactor(rc.Factor(value=factor))

    async def set_gravity(self, x=0, y=0, z=-9.8):
        if type(x) is tuple or type(x) is list:
            y = x[1]
            z = x[2]
            x = x[0]
        await self.rcs.SetGravity(msg.Coordinate(x=x,y=y,z=z))

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

    async def get_payload(self):
        res = await self.rcs.GetPayload(Empty())
        return ((res.cog.x, res.cog.y, res.cog.z), res.mass)

    async def set_payload_mass(self, mass):
        await self.rcs.SetPayloadMass(msg.PayloadMass(mass=mass))

    async def get_payload_mass(self):
        res = await self.rcs.GetPayloadMass(Empty())
        return res.mass

    async def set_payload_cog(self, x=0, y=0, z=0):
        if type(x) is tuple or type(x) is list:
            y = x[1]
            z = x[2]
            x = x[0]
        await self.rcs.SetPayloadCog(msg.PayloadCog(cog=msg.Coordinate(x=x,y=y,z=z)))

    async def get_payload_cog(self):
        res = await self.rcs.GetPayloadCog(Empty())
        return (res.cog.x, res.cog.y, res.cog.z)

    async def set_tcp(self, x=0, y=0, z=0, rz=0, ry=0, rx=0):
        tcp = CartesianPose(x, y, z, rz, ry, rx)
        await self.rcs.SetTcp(tcp.to_PR())

    async def get_tcp(self):
        res = await self.rcs.GetTcp(Empty())
        return CartesianPose(res)

    async def get_claw_aio(self, pin):
        pin = pin.lower()
        if pin == 'force':
            res = await self.rcs.GetClawHoldOn(Empty())
            return res.hold_on
        elif pin == 'weight':
            res = await self.rcs.GetClawWeight(Empty())
            return res.weight
        else: # pin == 'amplitude':
            res = await self.rcs.GetClawAmplitude(Empty())
            return res.amplitude

    async def set_claw_aio(self, pin, value=0):
        pin = pin.lower()
        if pin == 'force':
            await self.rcs.SetClawForce(rc.Force(force=value))
        else: # pin == 'amplitude':
            await self.rcs.SetClawAmplitude(rc.Amplitude(amplitude=value))

    async def set_claw(self, force=0, amplitude=0):
        # TODO: RC
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
            pose_is_joint_angle=getattr(p, 'is_joint', True),
            acceleration=a,
            velocity=v,
            time=t,
            blend_radius=r
        )
        if type(p) is CartesianPose:
            p.base_set_PR(req.pose_base)
        await self.rcs.MoveJ(req)

    async def movel(self, p, a=0, v=0, t=0, r=0):
        req = rc.MoveLRequest(
            pose_to = list(p.pos),
            pose_is_joint_angle=getattr(p, 'is_joint', False),
            acceleration=a,
            velocity=v,
            time=t,
            blend_radius=r
        )
        if type(p) is CartesianPose:
            p.base_set_PR(req.pose_base)
        await self.rcs.MoveL(req)

    async def movec(self, via, p, rad=0, a=0, v=0, t=0, r=0):
        req = rc.MoveCRequest(
            pose_via = list(via.pos),
            pose_via_is_joint=getattr(via, 'is_joint', False),
            pose_to = list(p.pos),
            pose_to_is_joint=getattr(p, 'is_joint', False),
            acceleration=a,
            velocity=v,
            time=t,
            blend_radius=r,
            rad=rad
        )
        if type(p) is CartesianPose:
            p.base_set_PR(req.pose_base)
        await self.rcs.MoveC(req)

    async def stop_move(self):
        await self.rcs.StopMove(Empty())

    async def movej_until(self, p, a=0, v=0, t=0, cb=None):
        pass

    async def movej_until_rt(self, p, a=0, v=0, t=0, logic="AND", io={}, cb=None):
        pass

    async def movel_until(self, p, a=0, v=0, t=0, cb=None):
        pass

    async def movel_until_rt(self, p, a=0, v=0, t=0, logic="AND", io={}, cb=None):
        pass

    async def movec_until(self, via, p, rad=0, a=0, v=0, t=0, cb=None):
        pass

    async def movec_until_rt(self, via, p, rad=0, a=0, v=0, t=0, logic="AND", io={}, cb=None):
        pass

    async def kinematics_forward(self, *p):
        j = JointPose(*p)
        res = await self.rcs.KinematicsForward(j.to_Joint())
        return CartesianPose(*res.vector)

    async def kinematics_inverse(self, *p):
        j = CartesianPose(*p)
        res = await self.rcs.KinematicsInverse(j.to_Vector())
        return JointPose(*res.joints)

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
        res = await self.rcs.GetTargetJointPositions(Empty())
        return JointPose(*res.joints)

    async def get_actual_joint_speeds(self):
        res = await self.rcs.GetActualJointSpeeds(Empty())
        return tuple(res.joints)

    async def get_target_joint_speeds(self):
        res = await self.rcs.GetTargetJointSpeeds(Empty())
        return tuple(res.joints)

    async def get_joint_torques(self):
        res = await self.rcs.GetJointTorques(Empty())
        return tuple(res.joints)

    async def get_actual_joint_torques(self):
        res = await self.rcs.GetRobotData(Empty())
        return tuple(res.actualTorque.joints)
        return ret

    async def get_target_joint_torques(self):
        res = await self.rcs.GetRobotData(Empty())
        return tuple(res.targetTorque.joints)

    async def get_joint_temperatures(self):
        res = await self.rcs.GetRobotData(Empty())
        return tuple(res.jointTemps.joints)

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
        res = await self.rcs.GetTargetTcpPose(Empty())
        return CartesianPose(*res.vector)

    async def get_robot_poses(self):
        res = await self.rcs.GetRobotData(Empty())
        ret = {}
        ret["actual_joint_pose"] = tuple(res.targetJoint.joints)
        ret["target_joint_pose"] = tuple(res.actualJoint.joints)
        ret["target_pose"] = tuple(res.targetTcpPose.vector)
        ret["actual_pose"] = tuple(res.actualTcpPose.vector)
        return ret

    async def get_robot_data(self):
        res = await self.rcs.GetRobotData(Empty())
        ret = {}
        ret["target_joint"] = tuple(res.targetJoint.joints)
        ret["actual_joint"] = tuple(res.actualJoint.joints)
        ret["target_pose"] = tuple(res.targetTcpPose.vector)
        ret["actual_pose"] = tuple(res.actualTcpPose.vector)
        ret["target_torque"] = tuple(res.targetTorque.joints)
        ret["actual_torque"] = tuple(res.actualTorque.joints)
        ret["target_vel"] = tuple(res.targetJointSpeed.joints)
        ret["actual_vel"] = tuple(res.actualJointSpeed.joints)
        ret["target_acc"] = tuple([]) # TODO: res.targetJointAcc.joints
        ret["actual_acc"] = tuple([]) # TODO: res.actualJointAcc.joints
        ret["temp"] = tuple(res.jointTemps.joints)
        return ret

    async def get_robot_io_data(self):
        res = await self.rcs.GetRobotIOData([rc.RobotDataCmd()]).read()
        ret = {}
        ret["di"] = tuple(map(lambda dio: {"pin": dio.pin, "value": dio.value}, res.robotDIOIn))
        ret["do"] = tuple(map(lambda dio: {"pin": dio.pin, "value": dio.value}, res.robotDIOOut))
        ret["ai"] = tuple(map(lambda aio: {"pin": aio.pin, "value": aio.value}, res.robotAIOIn))
        ret["ao"] = tuple(map(lambda aio: {"pin": aio.pin, "value": aio.value}, res.robotAIOOut))
        ret["flange_di"] = tuple(map(lambda dio: {"pin": dio.pin, "value": dio.value}, res.tcpDIOIn))
        ret["flange_do"] = tuple(map(lambda dio: {"pin": dio.pin, "value": dio.value}, res.tcpDIOOut))
        return ret

    async def set_do(self, pin, value):
        await self.rcs.SetDIO(msg.DIO(pin=pin, value=value))

    async def get_di(self, pin):
        res = await self.rcs.GetDIO(msg.IOPin(pin=pin))
        return res.value

    async def set_ao(self, pin, value):
        await self.rcs.SetAIO(msg.AIO(pin=pin, value=value))

    async def get_ai(self, pin):
        res = await self.rcs.GetAIO(msg.IOPin(pin=pin))
        return res.value

    async def set_ai_mode(self, pin, mode):
        await self.rcs.SetAInMode(msg.AIO(pin=pin, mode=mode))

    async def get_ai_mode(self, pin):
        res = await self.rcs.GetAInMode(msg.IOPin(pin=pin))
        return res.mode

    async def set_ao_mode(self, pin, mode):
        await self.rcs.SetAOutMode(msg.AIO(pin=pin, mode=mode))

    async def get_ao_mode(self, pin):
        res = await self.rcs.GetAOutMode(msg.IOPin(pin=pin))
        return res.mode

    async def set_flange_do(self, pin, value):
        await self.rcs.SetTcpDIO(msg.DIO(pin=pin, value=value))

    async def get_flange_di(self, pin):
        res = await self.rcs.GetTcpDIO(msg.IOPin(pin=pin))
        return res.value

    async def set_led(self, mode, speed, color=[]):
        await self.rcs.SetLED(msg.LEDStatus(mode=mode, speed=speed, color=color))

    async def set_voice(self, voice, volume):
        await self.rcs.SetVoice(msg.VoiceStatus(voice=voice, volume=volume))

    async def set_fan(self, on):
        await self.rcs.SetFan(msg.FanStatus(fan=on))

    async def set_signal(self, pin, value):
        await self.rcs.SetSignal(msg.SignalValue(index=pin, value=value))

    async def get_signal(self, pin):
        res = await self.rcs.GetSignal(msg.SignalValue(index=pin))
        return res.value

    async def add_signal(self, pin, delta):
        await self.rcs.AddSignal(msg.SignalValue(index=pin, value=value))

    async def enable_joint_limits(self):
        await self.pcs.EnableJointLimit(pc.TrueOrFalse(val=True))

    async def disable_joint_limits(self):
        await self.pcs.EnableJointLimit(pc.TrueOrFalse(val=False))

import grpc

from google.protobuf.empty_pb2 import Empty
from .pb2 import robot_controller_pb2_grpc
from .pb2 import private_controller_pb2_grpc

from .type import *

class LebaiRobot:
    '''
    :param ip: 机器人设备 IP
    :param sync: 非移动指令自动同步

    :returns: 返回一个乐白机器人控制实例
    '''
    def __init__(self, ip, sync=True):
        self.rcc = grpc.insecure_channel(f'{ip}:5181')
        self.rcs = robot_controller_pb2_grpc.RobotControllerStub(self.rcc)

        self.pcc = grpc.insecure_channel(f'{ip}:5182')
        self.pcs = private_controller_pb2_grpc.RobotPrivateControllerStub(self.pcc)

        self._sync_flag = sync

    def _sync(self):
        if self._sync_flag:
            self.sync()

    def sync(self):
        self.rcs.Sync(Empty())

    def start_sys(self):
        self._sync()
        self.rcs.StartSys(Empty())

    def stop_sys(self):
        self._sync()
        self.rcs.StopSys(Empty())

    def powerdown(self):
        self._sync()
        self.rcs.PowerDown(Empty)

    def stop(self):
        self._sync()
        self.rcs.Stop(Empty())

    def estop(self):
        self._sync()
        self.rcs.EStop(Empty())

    def teach_mode(self):
        self._sync()
        self.rcs.TeachMode(Empty)

    def end_teach_mode(self):
        self._sync()
        self.rcs.EndTeachMode(Empty)

    def resume(self):
        self._sync()
        self.rcs.Resume(Empty())

    def pause(self):
        self._sync()
        self.rcs.Pause(Empty())

    def get_robot_mode(self):
        self._sync()
        res = self.rcs.GetRobotMode(Empty())
        return RobotState(res.mode)

    def get_velocity_factor(self):
        self._sync()
        res = self.rcs.GetVelocityFactor(Empty())
        return res.value

    def set_velocity_factor(self, factor):
        self._sync()
        self.rcs.SetVelocityFactor(rc.Factor(value=factor))
        self._sync()

    def set_gravity(self, x=0, y=0, z=-9.8):
        if type(x) is tuple or type(x) is list:
            y = x[1]
            z = x[2]
            x = x[0]
        self._sync()
        self.rcs.SetGravity(msg.Coordinate(x=x,y=y,z=z))

    def get_gravity(self):
        self._sync()
        res = self.rcs.GetGravity(Empty())
        return (res.x, res.y, res.z)

    def set_payload(self, x=0, y=0, z=-9.8, mass=0):
        if type(x) is tuple:
            if type(y) is not tuple and type(x[0]) is tuple:
                y = x[1]
                x = x[0]
            mass = y
            z = x[2]
            y = x[1]
            x = x[0]
        self._sync()
        self.rcs.SetPayload(msg.Payload(mass=mass, cog=msg.Coordinate(x=x,y=y,z=z)))

    def get_payload(self):
        self._sync()
        res = self.rcs.GetPayload(Empty())
        return ((res.cog.x, res.cog.y, res.cog.z), res.mass)

    def set_payload_mass(self, mass):
        self._sync()
        self.rcs.SetPayloadMass(msg.PayloadMass(mass=mass))

    def get_payload_mass(self):
        self._sync()
        res = self.rcs.GetPayloadMass(Empty())
        return res.mass

    def set_payload_cog(self, x=0, y=0, z=0):
        if type(x) is tuple or type(x) is list:
            y = x[1]
            z = x[2]
            x = x[0]
        self._sync()
        self.rcs.SetPayloadCog(msg.PayloadCog(cog=msg.Coordinate(x=x,y=y,z=z)))

    def get_payload_cog(self):
        self._sync()
        res = self.rcs.GetPayloadCog(Empty())
        return (res.cog.x, res.cog.y, res.cog.z)

    def set_tcp(self, x=0, y=0, z=0, rz=0, ry=0, rx=0):
        self._sync()
        tcp = CartesianPose(x, y, z, rz, ry, rx)
        self.rcs.SetTcp(tcp.to_PR())

    def get_tcp(self):
        self._sync()
        res = self.rcs.GetTcp(Empty())
        return CartesianPose(res)

    def get_claw_aio(self, pin):
        self._sync()
        pin = pin.lower()
        if pin == 'force':
            res = self.rcs.GetClawHoldOn(Empty())
            return res.hold_on
        elif pin == 'weight':
            res = self.rcs.GetClawWeight(Empty())
            return res.weight
        else: # pin == 'amplitude':
            res = self.rcs.GetClawAmplitude(Empty())
            return res.amplitude

    def set_claw_aio(self, pin, value=0):
        self._sync()
        pin = pin.lower()
        if pin == 'force':
            self.rcs.SetClawForce(rc.Force(force=value))
        else: # pin == 'amplitude':
            self.rcs.SetClawAmplitude(rc.Amplitude(amplitude=value))

    def set_claw(self, force=0, amplitude=0):
        self._sync()
        self.rcs.SetClawForce(rc.Force(force=force))
        self.rcs.SetClawAmplitude(rc.Amplitude(amplitude=amplitude))

    def movej(self, p, a=0, v=0, t=0, r=0):
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
        self.rcs.MoveJ(req)

    def movel(self, p, a=0, v=0, t=0, r=0):
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
        self.rcs.MoveL(req)

    def movec(self, via, p, rad=0, a=0, v=0, t=0, r=0):
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
        self.rcs.MoveC(req)

    def stop_move(self):
        self.rcs.StopMove(Empty())

    def movej_until(self, p, a=0, v=0, t=0, cb=None):
        pass

    def movej_until_rt(self, p, a=0, v=0, t=0, logic="AND", io={}, cb=None):
        pass

    def movel_until(self, p, a=0, v=0, t=0, cb=None):
        pass

    def movel_until_rt(self, p, a=0, v=0, t=0, logic="AND", io={}, cb=None):
        pass

    def movec_until(self, via, p, rad=0, a=0, v=0, t=0, cb=None):
        pass

    def movec_until_rt(self, via, p, rad=0, a=0, v=0, t=0, logic="AND", io={}, cb=None):
        pass

    def kinematics_forward(self, *p):
        j = JointPose(*p)
        res = self.rcs.KinematicsForward(j.to_Joint())
        return CartesianPose(*res.vector)

    def kinematics_inverse(self, *p):
        j = CartesianPose(*p)
        res = self.rcs.KinematicsInverse(j.to_Vector())
        return JointPose(*res.joints)

    def pose_times(self):
        pass

    def pose_inverse(self):
        pass

    def get_actual_joint_positions(self):
        '''获取实际关节位置

        :returns: `JointPose` 关节位置
        '''
        self._sync()
        res = self.rcs.GetActualJointPositions(Empty())
        return JointPose(*res.joints)

    def get_target_joint_positions(self):
        self._sync()
        res = self.rcs.GetTargetJointPositions(Empty())
        return JointPose(*res.joints)

    def get_actual_joint_speeds(self):
        self._sync()
        res = self.rcs.GetActualJointSpeeds(Empty())
        return tuple(res.joints)

    def get_target_joint_speeds(self):
        self._sync()
        res = self.rcs.GetTargetJointSpeeds(Empty())
        return tuple(res.joints)

    def get_joint_torques(self):
        self._sync()
        res = self.rcs.GetJointTorques(Empty())
        return tuple(res.joints)

    def get_actual_joint_torques(self):
        self._sync()
        res = self.rcs.GetRobotData(Empty())
        return tuple(res.actualTorque.joints)

    def get_target_joint_torques(self):
        self._sync()
        res = self.rcs.GetRobotData(Empty())
        return tuple(res.targetTorque.joints)

    def get_joint_temperatures(self):
        self._sync()
        res = self.rcs.GetRobotData(Empty())
        return tuple(res.jointTemps.joints)

    def get_joint_temp(self, joint):
        '''获取关节温度

        :param joint: 关节序号

        :returns: 关节当前温度 (℃)
        '''
        self._sync()
        res = self.rcs.GetJointTemp(rc.IntRequest(index=joint))
        return res.degree

    def get_actual_tcp_pose(self):
        '''获取实际空间位置

        :returns: `CartesianPose` 空间位置
        '''
        self._sync()
        res = self.rcs.GetActualTcpPose(Empty())
        return CartesianPose(*res.vector)

    def get_target_tcp_pose(self):
        self._sync()
        res = self.rcs.GetTargetTcpPose(Empty())
        return CartesianPose(*res.vector)

    def get_robot_poses(self):
        self._sync()
        res = self.rcs.GetRobotData(Empty())
        ret = {}
        ret["actual_joint_pose"] = tuple(res.targetJoint.joints)
        ret["target_joint_pose"] = tuple(res.actualJoint.joints)
        ret["target_pose"] = tuple(res.targetTcpPose.vector)
        ret["actual_pose"] = tuple(res.actualTcpPose.vector)
        return ret

    def get_robot_data(self):
        self._sync()
        res = self.rcs.GetRobotData(Empty())
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

    def _generate_robot_data_cmd(self, n=1):
        for i in range(0, n):
            yield rc.RobotDataCmd()

    def get_robot_io_data(self):
        self._sync()
        res = self.rcs.GetRobotIOData(self._generate_robot_data_cmd())
        for io in res:
            ret = {}
            ret["di"] = tuple(map(lambda dio: {"pin": dio.pin, "value": dio.value}, io.robotDIOIn))
            ret["do"] = tuple(map(lambda dio: {"pin": dio.pin, "value": dio.value}, io.robotDIOOut))
            ret["ai"] = tuple(map(lambda aio: {"pin": aio.pin, "value": aio.value}, io.robotAIOIn))
            ret["ao"] = tuple(map(lambda aio: {"pin": aio.pin, "value": aio.value}, io.robotAIOOut))
            ret["flange_di"] = tuple(map(lambda dio: {"pin": dio.pin, "value": dio.value}, io.tcpDIOIn))
            ret["flange_do"] = tuple(map(lambda dio: {"pin": dio.pin, "value": dio.value}, io.tcpDIOOut))
            return ret

    def set_do(self, pin, value):
        self._sync()
        self.rcs.SetDIO(msg.DIO(pin=pin, value=value))

    def get_di(self, pin):
        self._sync()
        res = self.rcs.GetDIO(msg.IOPin(pin=pin))
        return res.value

    def set_ao(self, pin, value):
        self._sync()
        self.rcs.SetAIO(msg.AIO(pin=pin, value=value))

    def get_ai(self, pin):
        self._sync()
        res = self.rcs.GetAIO(msg.IOPin(pin=pin))
        return res.value

    def set_ai_mode(self, pin, mode):
        self._sync()
        self.rcs.SetAInMode(msg.AIO(pin=pin, mode=mode))

    def get_ai_mode(self, pin):
        self._sync()
        res = self.rcs.GetAInMode(msg.IOPin(pin=pin))
        return res.mode

    def set_ao_mode(self, pin, mode):
        self._sync()
        self.rcs.SetAOutMode(msg.AIO(pin=pin, mode=mode))

    def get_ao_mode(self, pin):
        self._sync()
        res = self.rcs.GetAOutMode(msg.IOPin(pin=pin))
        return res.mode

    def set_flange_do(self, pin, value):
        self._sync()
        self.rcs.SetTcpDIO(msg.DIO(pin=pin, value=value))

    def get_flange_di(self, pin):
        self._sync()
        res = self.rcs.GetTcpDIO(msg.IOPin(pin=pin))
        return res.value

    def set_led(self, mode, speed, color=[]):
        self._sync()
        self.rcs.SetLED(msg.LEDStatus(mode=mode, speed=speed, color=color))

    def set_voice(self, voice, volume):
        self._sync()
        self.rcs.SetVoice(msg.VoiceStatus(voice=voice, volume=volume))

    def set_fan(self, on):
        self._sync()
        self.rcs.SetFan(msg.FanStatus(fan=on))

    def set_signal(self, pin, value):
        self._sync()
        self.rcs.SetSignal(msg.SignalValue(index=pin, value=value))

    def get_signal(self, pin):
        self._sync()
        res = self.rcs.GetSignal(msg.SignalValue(index=pin))
        return res.value

    def add_signal(self, pin, delta):
        self._sync()
        self.rcs.AddSignal(msg.SignalValue(index=pin, value=value))

    def enable_joint_limits(self):
        self._sync()
        self.pcs.EnableJointLimit(pc.TrueOrFalse(val=True))

    def disable_joint_limits(self):
        self._sync()
        self.pcs.EnableJointLimit(pc.TrueOrFalse(val=False))

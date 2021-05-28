from typing import Optional, Iterator

import grpc
from google.protobuf.empty_pb2 import Empty

from lebai.lebai_http_service import LebaiHttpService
from lebai.pb2 import private_controller_pb2_grpc
from lebai.pb2 import robot_controller_pb2_grpc
from .pb2.private_controller_pb2 import TrueOrFalse
from .type import *


class LebaiRobot:
    def __init__(self, ip, sync=True):
        """

        :ip: 机器人设备 IP
        :sync: 非移动指令自动同步
        """
        self.ip = ip
        self.rcc = grpc.insecure_channel(f'{ip}:5181')
        self.rcs = robot_controller_pb2_grpc.RobotControllerStub(self.rcc)

        self.pcc = grpc.insecure_channel(f'{ip}:5182')
        self.pcs = private_controller_pb2_grpc.RobotPrivateControllerStub(self.pcc)
        self.http_service = LebaiHttpService(ip)

        self._sync_flag = sync

    def is_connected(self) -> bool:
        """
        是否连接

        :return:
        """
        try:
            return self.get_robot_mode() != RobotState.DISCONNECTED
        except grpc.RpcError:
            return False

    def _sync(self):
        if self._sync_flag:
            self.sync()

    def sync(self) -> None:
        """
        同步，等待指定命令执行完成

        """
        try:
            self.rcs.Sync(Empty())
        except grpc.RpcError:
            pass

    def sleep(self, time: int) -> None:
        """
        等待

        :param time: 等待时间,单位毫秒
        """
        try:
            self.rcs.Sleep(rc.SleepRequest(time=time))
        except grpc.RpcError:
            pass

    def start_sys(self) -> None:
        """
        启动

        """
        # self._sync()
        self.rcs.StartSys(Empty())

    def stop_sys(self) -> None:
        """
        关闭

        """

        # self._sync()
        self.rcs.StopSys(Empty())

    def powerdown(self) -> None:
        """
        关闭电源

        """

        # self._sync()
        self.rcs.PowerDown(Empty())

    def stop(self) -> None:
        """
        停止程序

        """

        # self._sync()
        self.rcs.Stop(Empty())

    def estop(self) -> None:
        """
        急停

        """
        # self._sync()
        self.rcs.EStop(Empty())

    def teach_mode(self) -> None:
        """
        开启示教模式

        """

        self._sync()
        self.rcs.TeachMode(Empty())

    def end_teach_mode(self) -> None:
        """
        关闭示教模式

        """

        self._sync()
        self.rcs.EndTeachMode(Empty())

    def resume(self) -> None:
        """
        恢复机器人

        """
        # self._sync()
        self.rcs.Resume(Empty())

    def pause(self) -> None:
        """
        暂停机器人

        """
        # self._sync()
        self.rcs.Pause(Empty())

    def get_robot_mode(self) -> RobotState:
        """
        获取机器人状态

        :return: 机器人状态
        """
        self._sync()
        res = self.rcs.GetRobotMode(Empty())
        return RobotState(res.mode)

    def get_velocity_factor(self) -> float:
        """
        获取速度因子

        :return: 速度因子
        """
        self._sync()
        res = self.rcs.GetVelocityFactor(Empty())
        return res.value

    def set_velocity_factor(self, factor) -> None:
        """
        设置速度因子

        :param factor: 速度因子
        """
        self._sync()
        self.rcs.SetVelocityFactor(rc.Factor(value=factor))
        self._sync()

    def set_gravity(self, x: float = 0, y: float = 0, z: float = -9.8) -> None:
        """
        设置重力

        :param x:
        :param y:
        :param z:
        """
        if type(x) is tuple or type(x) is list:
            y = x[1]
            z = x[2]
            x = x[0]
        self._sync()
        self.rcs.SetGravity(msg.Coordinate(x=x, y=y, z=z))

    def get_gravity(self) -> (float, float, float):
        """
        获取重力

        :return:
        """
        self._sync()
        res = self.rcs.GetGravity(Empty())
        return (res.x, res.y, res.z)

    def set_payload(self, x: float = 0, y: float = 0, z: float = -9.8, mass: float = 0) -> None:
        """
        设置负荷

        :param x:
        :param y:
        :param z:
        :param mass:
        """
        if type(x) is tuple:
            if type(y) is not tuple and type(x[0]) is tuple:
                y = x[1]
                x = x[0]
            mass = y
            z = x[2]
            y = x[1]
            x = x[0]
        self._sync()
        self.rcs.SetPayload(msg.Payload(mass=mass, cog=msg.Coordinate(x=x, y=y, z=z)))

    def get_payload(self) -> ((float, float, float), float):
        """
        获取负荷

        :return:
        """
        self._sync()
        res = self.rcs.GetPayload(Empty())
        return ((res.cog.x, res.cog.y, res.cog.z), res.mass)

    def set_payload_mass(self, mass: float) -> None:
        """
        设置负荷的质量

        :param mass:
        :type mass:
        """
        self._sync()
        self.rcs.SetPayloadMass(msg.PayloadMass(mass=mass))

    def get_payload_mass(self) -> float:
        """
        获取负荷的质量

        :return:
        """
        self._sync()
        res = self.rcs.GetPayloadMass(Empty())
        return res.mass

    def set_payload_cog(self, x: float = 0, y: float = 0, z: float = 0) -> None:
        """
        设置负荷的质心

        :param x:
        :param y:
        :param z:
        """
        if type(x) is tuple or type(x) is list:
            y = x[1]
            z = x[2]
            x = x[0]
        self._sync()
        self.rcs.SetPayloadCog(msg.PayloadCog(cog=msg.Coordinate(x=x, y=y, z=z)))

    def get_payload_cog(self) -> (float, float, float):
        """
        获取负荷的质心

        :return:
        """
        self._sync()
        res = self.rcs.GetPayloadCog(Empty())
        return (res.cog.x, res.cog.y, res.cog.z)

    def set_tcp(self, x: float = 0, y: float = 0, z: float = 0, rz: float = 0, ry: float = 0, rx: float = 0) -> None:
        """
        设置TCP

        :param x:
        :param y:
        :param z:
        :param rz:
        :param ry:
        :param rx:
        """
        self._sync()
        tcp = CartesianPose(x, y, z, rz, ry, rx)
        self.rcs.SetTcp(tcp._to_PR())

    def get_tcp(self) -> CartesianPose:
        """
        获取TCP

        :return:
        """
        self._sync()
        res = self.rcs.GetTcp(Empty())
        return CartesianPose(res)

    def get_claw_aio(self, pin: int) -> float:
        """
        获取手爪参数

        :param pin:
        :return:
        """
        self._sync()
        pin = pin.lower()
        if pin == 'force':
            res = self.rcs.GetClawHoldOn(Empty())
            return res.hold_on
        elif pin == 'weight':
            res = self.rcs.GetClawWeight(Empty())
            return res.weight
        else:  # pin == 'amplitude':
            res = self.rcs.GetClawAmplitude(Empty())
            return res.amplitude

    def set_claw_aio(self, pin: int, value: float = 0) -> None:
        """
        设置手爪参数

        :param pin:
        :param value:
        """
        self._sync()
        pin = pin.lower()
        if pin == 'force':
            self.rcs.SetClawForce(rc.Force(force=value))
        else:  # pin == 'amplitude':
            self.rcs.SetClawAmplitude(rc.Amplitude(amplitude=value))

    def set_claw(self, force: float = 0, amplitude: float = 0) -> None:
        """
        设置手爪

        :param force:
        :param amplitude:
        """
        self._sync()
        self.rcs.SetClawForce(rc.Force(force=force))
        self.rcs.SetClawAmplitude(rc.Amplitude(amplitude=amplitude))

    def movej(self, p, a=0, v=0, t=0, r=0, is_joint=None):
        """
        线性移动（关节空间）

        :is_joint:
        :p: `JointPose` 关节位置，`CartesianPose` 空间位置（将通过运动学反解转为关节位置）
        :a: 主轴的关节加速度 (rad/s\ :sup:`2`)
        :v: 主轴的关节速度 (rad/s)
        :t: 运动时间 (s)
        :r: 交融半径 (m)
        """
        if is_joint is None:
            is_joint = getattr(p, 'is_joint', True)
        if not hasattr(p, 'pos'):
            p = JointPose(*p)
        req = rc.MoveJRequest(
            joint_pose_to=list(p.pos),
            pose_is_joint_angle=is_joint,
            acceleration=a,
            velocity=v,
            time=t,
            blend_radius=r
        )
        if type(p) is CartesianPose:
            p._base_set_PR(req.pose_base)
        self.rcs.MoveJ(req)

    def movel(self, p, a=0, v=0, t=0, r=0, is_joint=None):
        """
        线性移动（工具空间）

        :p: `JointPose` 关节位置，`CartesianPose` 空间位置（将通过运动学反解转为关节位置）
        :a: 轴的关节加速度 (rad/s\ :sup:`2`)
        :v: 主轴的关节速度 (rad/s)
        :t: 运动时间 (s)
        :r: 交融半径 (m)
        :is_joint:
        """
        if is_joint is None:
            is_joint = getattr(p, 'is_joint', False)
        if not hasattr(p, 'pos'):
            p = CartesianPose(*p)
        req = rc.MoveLRequest(
            pose_to=list(p.pos),
            pose_is_joint_angle=is_joint,
            acceleration=a,
            velocity=v,
            time=t,
            blend_radius=r
        )
        if type(p) is CartesianPose:
            p._base_set_PR(req.pose_base)
        self.rcs.MoveL(req)

    def movec(self, via, p, rad=0, a=0, v=0, t=0, r=0, is_joint=None):
        """
        圆弧移动（工具空间）

        :param via:
        :param p:
        :param rad:
        :param a:
        :param v:
        :param t:
        :param r:
        :param is_joint:
        """
        if not hasattr(p, 'pos'):
            p = CartesianPose(*p)
        if not hasattr(via, 'pos'):
            via = CartesianPose(*via)
        req = rc.MoveCRequest(
            pose_via=list(via.pos),
            pose_via_is_joint=is_joint if is_joint is not None else getattr(via, 'is_joint', True),
            pose_to=list(p.pos),
            pose_to_is_joint=is_joint if is_joint is not None else getattr(p, 'is_joint', True),
            acceleration=a,
            velocity=v,
            time=t,
            blend_radius=r,
            rad=rad
        )
        if type(p) is CartesianPose:
            p._base_set_PR(req.pose_base)
        self.rcs.MoveC(req)

    def stop_move(self) -> None:
        """
        停止当前移动

        """
        self.rcs.StopMove(Empty())

    def move_pvat(self, p: list, v: list, a: list, t: float) -> None:
        """
        指定位置、速度、加速度、时间的伺服移动

        :param p: 关节位置列表 (rad)，类型：list[float]
        :param v: 关节速度列表 (rad/s)，类型：list[float]
        :param a: 关节加速度列表 (rad/s^2)，类型：list[float]
        :param t: 总运动时间 (s)
        """
        self.rcs.MovePVAT(rc.PVATRequest(duration=t, q=p, v=v, acc=a))

    def move_pvats(self, pvt_iter: list) -> None:
        """

        @param pvt_iter: 类型：list[PVAT]
        @type pvt_iter: list[PVAT]
        """
        self.rcs.MovePVATStream((rc.PVATRequest(duration=s.duration, q=s.q, v=s.v, acc=s.acc) for s in pvt_iter))

    def move_pvt(self, p: list, v: list, t: float)->None:
        """
        指定位置、速度、时间的伺服移动, 加速度将自动计算。

        :param p: 关节位置列表 (rad)，类型：list[float],
        :param v: 关节速度列表 (rad/s)，类型：list[float],
        :param t: 总运动时间 (s)
        """
        self.rcs.MovePVT(rc.PVATRequest(duration=t, q=p, v=v))

    def move_pvts(self, pvt_iter: list):
        """
        指定位置、速度、时间的伺服移动, 加速度将自动计算。

        :param pvt_iter: 类型：list[PVAT]
        :return:
        """
        self.rcs.MovePVTStream((rc.PVATRequest(duration=s.duration, q=s.q, v=s.v) for s in pvt_iter))

    def move_pt(self, p: list, t: float) -> None:
        """
        指定位置和时间的伺服移动,速度和加速度将自动计算。

        :param p: 关节位置列表 (rad)，类型：list[float]
        :param t: 总运动时间 (s)
        """
        self.rcs.MovePT(rc.PVATRequest(duration=t, q=p))

    def move_pts(self, pt_iter: list):
        """
        指定位置和时间的伺服移动,速度和加速度将自动计算。

        :param pt_iter: 类型：list[PVAT]
        :return:
        """
        self.rcs.MovePTStream((rc.PVATRequest(duration=s.duration, q=s.q) for s in pt_iter))

    def movej_until(self, p, a=0, v=0, t=0, cb=None):
        """
        todo: 待实现

        """
        pass

    def movej_until_rt(self, p, a=0, v=0, t=0, logic='AND', io={}, cb=None):
        """
        todo: 待实现

        """
        pass

    def movel_until(self, p, a=0, v=0, t=0, cb=None):
        """
        todo: 待实现

        """
        pass

    def movel_until_rt(self, p, a=0, v=0, t=0, logic='AND', io={}, cb=None):
        """
        todo: 待实现

        """
        pass

    def movec_until(self, via, p, rad=0, a=0, v=0, t=0, cb=None):
        """
        todo: 待实现

        """
        pass

    def movec_until_rt(self, via, p, rad=0, a=0, v=0, t=0, logic='AND', io={}, cb=None):
        """
        todo: 待实现

        """
        pass

    def kinematics_forward(self, *p: list):
        """
        机器人正解

        :param p: 类型：list[float]
        :return:
        """
        j = JointPose(*p)
        res = self.rcs.KinematicsForward(j._to_Joint())
        return CartesianPose(*res.vector)

    def kinematics_inverse(self, *p: list):
        """
        机器人反解

        :param p:  类型：list[float]
        :return:
        """
        j = CartesianPose(*p)
        res = self.rcs.KinematicsInverse(j._to_Vector())
        return JointPose(*res.joints)

    def pose_times(self) -> None:
        """
        todo: 待实现

        """
        pass

    def pose_inverse(self) -> None:
        """
        todo: 待实现

        """
        pass

    def get_actual_joint_positions(self) -> JointPose:
        """
        获取实际关节位置

        :returns: `JointPose` 关节位置
        """
        self._sync()
        res = self.rcs.GetActualJointPositions(Empty())
        return JointPose(*res.joints)

    def get_target_joint_positions(self) -> JointPose:
        """
        获得所有关节的期望角度位置

        :return: 所有关节的期望角度位置
        """
        self._sync()
        res = self.rcs.GetTargetJointPositions(Empty())
        return JointPose(*res.joints)

    def get_actual_joint_speeds(self) -> tuple:
        """
        获得所有关节的实际角速度

        :return: 所有关节的实际角速度
        """
        self._sync()
        res = self.rcs.GetActualJointSpeeds(Empty())
        return tuple(res.joints)

    def get_target_joint_speeds(self) -> tuple:
        """
        获得所有关节的期望角速度

        :return: 所有关节的期望角速度
        """
        self._sync()
        res = self.rcs.GetTargetJointSpeeds(Empty())
        return tuple(res.joints)

    def get_joint_torques(self) -> tuple:
        """
        获得每个关节的扭矩值

        :return: 每个关节的扭矩值
        """
        self._sync()
        res = self.rcs.GetJointTorques(Empty())
        return tuple(res.joints)

    def get_actual_joint_torques(self) -> tuple:
        """
        获取实际力矩

        :return:
        :rtype:
        """
        self._sync()
        res = self.rcs.GetRobotData(Empty())
        return tuple(res.actualTorque.joints)

    def get_target_joint_torques(self) -> tuple:
        """
        获取理论力矩

        :return:
        """
        self._sync()
        res = self.rcs.GetRobotData(Empty())
        return tuple(res.targetTorque.joints)

    def get_joint_temperatures(self) -> tuple:
        """
        获取关节温度

        :return:
        """
        self._sync()
        res = self.rcs.GetRobotData(Empty())
        return tuple(res.jointTemps.joints)

    def get_joint_temp(self, joint: int) -> float:
        """
        获取关节温度

        :param joint: 关节序号

        :returns: 关节当前温度 (℃)
        """
        self._sync()
        res = self.rcs.GetJointTemp(rc.IntRequest(index=joint))
        return res.degree

    def get_actual_tcp_pose(self) -> CartesianPose:
        """
        获取实际空间位置

        :returns: `CartesianPose` 空间位置
        """
        self._sync()
        res = self.rcs.GetActualTcpPose(Empty())
        return CartesianPose(*res.vector)

    def get_target_tcp_pose(self) -> CartesianPose:
        """
        获得TCP的期望的姿态/位置

        :return: TCP的期望的姿态/位置
        """
        self._sync()
        res = self.rcs.GetTargetTcpPose(Empty())
        return CartesianPose(*res.vector)

    def get_robot_poses(self) -> RobotPoseData:
        """
        获取机器人姿态信息

        :return: 机器人姿态信息
        """
        self._sync()
        res = self.rcs.GetRobotData(Empty())
        return RobotPoseData(res)

    def get_robot_data(self) -> RobotData:
        """
        获取机器人数据

        :return: 机器人数据
        """
        self._sync()
        res = self.rcs.GetRobotData(Empty())
        return RobotData(res)

    def _generate_robot_data_cmd(self, n=1):
        for i in range(0, n):
            yield rc.RobotDataCmd()

    def get_robot_io_data(self) -> Iterator[RobotIOData]:
        """
        获取机器人IO数据

        :return: 机器人IO数据
        """
        self._sync()
        res = self.rcs.GetRobotIOData(self._generate_robot_data_cmd())
        for io in res:
            yield RobotIOData(io)

    def set_do(self, pin: int, value: int) -> None:
        """
        设置数字输出

        :param pin: 针脚
        :param value: 值
        """
        self._sync()
        self.rcs.SetDIO(msg.DIO(pin=pin, value=value))

    def get_di(self, pin: int) -> int:
        """
        获取数字输入

        :param pin: 针脚
        :return:
        """
        self._sync()
        res = self.rcs.GetDIO(msg.IOPin(pin=pin))
        return res.value

    def set_ao(self, pin: int, value: float) -> None:
        """
        设置模拟输出

        :param pin: 针脚
        :param value: 值
        """
        self._sync()
        self.rcs.SetAIO(msg.AIO(pin=pin, value=value))

    def get_ai(self, pin: int) -> float:
        """
        获取模拟输入

        :param pin: 针脚
        :return:
        """
        self._sync()
        res = self.rcs.GetAIO(msg.IOPin(pin=pin))
        return res.value

    def set_ai_mode(self, pin: int, mode: int) -> None:
        """
        设置模拟输入端口工作模式

        :param pin: 针脚
        :param mode:  0:电压，1:电流
        """
        self._sync()
        self.rcs.SetAInMode(msg.AIO(pin=pin, mode=mode))

    def get_ai_mode(self, pin: int) -> int:
        """
        获取模拟输入端口工作模式

        :param pin: 针脚
        """
        self._sync()
        res = self.rcs.GetAInMode(msg.IOPin(pin=pin))
        return res.mode

    def set_ao_mode(self, pin: int, mode: int) -> None:
        """
        设置模拟输出端口工作模式

        :param pin: 针脚
        :param mode:  0:电压，1:电流
        """
        self._sync()
        self.rcs.SetAOutMode(msg.AIO(pin=pin, mode=mode))

    def get_ao_mode(self, pin: int) -> int:
        """
        获取模拟输出端口工作模式

        :param pin: 针脚
        :return: 0:电压，1:电流
        """
        self._sync()
        res = self.rcs.GetAOutMode(msg.IOPin(pin=pin))
        return res.mode

    def set_flange_do(self, pin: int, value: int) -> None:
        """
        设置法兰数字输出

        :param pin:
        :param value:
        """
        self._sync()
        self.rcs.SetTcpDIO(msg.DIO(pin=pin, value=value))

    def get_flange_di(self, pin: int) -> int:
        """
        获取法兰数字输出

        :param pin:
        :return:
        """
        self._sync()
        res = self.rcs.GetTcpDIO(msg.IOPin(pin=pin))
        return res.value

    def set_led(self, mode: int, speed: float, color: list) -> None:
        """
        设置LED灯状态

        :param mode: 1：关闭、2：常亮、3：呼吸、4：均分旋转、5：同色旋转、6：闪烁
        :param speed: 速度 分三个等级，1：快速、2：正常、3：慢速
        :param color: 颜色，最多包含4个 0 ~ 15 之间的整数，类型：list[int]
        """
        self._sync()
        self.rcs.SetLED(msg.LEDStatus(mode=mode, speed=speed, color=color))

    def set_voice(self, voice: int, volume: int) -> None:
        """
        设置声音

        :voice: 声音列表 0~10
        :volume: 音量 分四个等级，0：静音、1：低、2：正常、3：高
        """
        self._sync()
        self.rcs.SetVoice(msg.VoiceStatus(voice=voice, volume=volume))

    def set_fan(self, fan: int) -> None:
        """
        设置风扇

        :param fan: 1：关闭、 2：开启
        """
        self._sync()
        self.rcs.SetFan(msg.FanStatus(fan=fan))

    def set_signal(self, pin: int, value: int) -> None:
        """
        设置信号

        :param pin: 针脚
        :param value:  值
        """
        self._sync()
        self.rcs.SetSignal(msg.SignalValue(index=pin, value=value))

    def get_signal(self, pin: int) -> int:
        """
        获取信号

        :param pin: 针脚
        """
        self._sync()
        res = self.rcs.GetSignal(msg.SignalValue(index=pin))
        return res.value

    def add_signal(self, pin: int, value: int) -> None:
        """
        添加信号

        :pin: 针脚
        :value:
        """
        self._sync()
        self.rcs.AddSignal(msg.SignalValue(index=pin, value=value))

    def enable_joint_limits(self) -> None:
        """
        启用关节限位检测

        """
        self._sync()
        self.pcs.EnableJointLimit(TrueOrFalse(val=True))

    def disable_joint_limits(self) -> None:
        """
        关闭关节限位检测

        """
        self._sync()
        self.pcs.EnableJointLimit(TrueOrFalse(val=False))

    def run_scene(self, scene_id: int, execute_count: int = 1, clear: bool = True) -> int:
        """
        运行场景

        :param scene_id: 场景Id
        :param execute_count: 执行数量，0 表示一直循环运行
        :param clear: 是否强制关闭正在运行的任务
        :return:
        """
        return self.http_service.run_scene(scene_id, execute_count, clear)['id']

    def rerun_task(self, task_id: int, execute_count: int = 1, clear: bool = True) -> int:
        """
        运行任务

        :param task_id: 任务Id
        :param execute_count: 执行数量，0 表示一直循环运行
        :param clear: 是否强制关闭正在运行的任务
        :return:
        """
        return self.http_service.run_task(task_id, execute_count, clear)['id']

    def execute_lua_code(self, task_name: str, code: str, execute_count: int = 1, clear: bool = True) -> int:
        """

        :param task_name: 任务名称
        :param code: lua 代码
        :param execute_count: 执行数量，0 表示一直循环运行
        :param clear: 是否强制关闭正在运行的任务
        :return:
        """
        return self.http_service.execute_lua_code(task_name, execute_count, clear, code)['id']

    def get_task(self, id: int) -> Optional[TaskInfo]:
        """
        获取任务信息

        :param id: 任务Id
        :return:
        """
        return self.http_service.get_task(id)

    def get_tasks(self, pi: int, ps: int) -> Optional[TasksResult]:
        """
        获取任务列表

        :param pi: 页码
        :param ps: 页大小
        :return:
        """
        return self.http_service.get_tasks(pi, ps)

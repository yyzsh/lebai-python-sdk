from typing import Optional, Iterator

from .type import *
from .client import JsonRpcClient


class LebaiRobot(JsonRpcClient):
    def __init__(self, ip, sync=True, port=3031):
        super().__init__(ip, port)

    def is_connected(self):
        return self.get_robot_mode() != RobotState.DISCONNECTED

    def sync(self):
        return self.send('sync')

    def wait(self, time):
        return self.send('wait', [time])

    def sleep(self, time):
        return self.wait(time)

    def start_sys(self):
        return self.send('start_sys')

    def stop_sys(self):
        return self.send('stop_sys')

    def powerdown(self):
        return self.send('powerdown')

    def stop(self):
        return self.send('stop')

    def estop(self):
        return self.send('estop')

    def teach_mode(self):
        return self.send('teach_mode')

    def end_teach_mode(self):
        return self.send('end_teach_mode')

    def resume(self):
        return self.send('resume')

    def pause(self, time = None):
        self.send('pause', [time])

    def get_robot_mode(self):
        return RobotState(self.send('get_robot_mode'))

    def get_velocity_factor(self):
        return self.send('get_velocity_factor')

    def set_velocity_factor(self, factor):
        return self.send('set_velocity_factor', [factor])

    def set_gravity(self, x = 0, y = 0, z = -9.8):
        if type(x) is tuple or type(x) is list:
            y = x[1]
            z = x[2]
            x = x[0]
        return self.send('set_gravity', [{'x': x, 'y': y, 'z': z}])

    def get_gravity(self):
        return self.send('get_gravity')

    def set_payload(self, x = 0, y = 0, z = -9.8, mass = 0):
        if type(x) is tuple:
            if type(y) is not tuple and type(x[0]) is tuple:
                y = x[1]
                x = x[0]
            mass = y
            z = x[2]
            y = x[1]
            x = x[0]
        self.send('set_payload', [mass, {'x': x, 'y': y, 'z': z}])

    def get_payload(self):
        return self.send('get_payload')

    def set_payload_mass(self, mass):
        return self.send('set_payload_mass', [mass])

    def get_payload_mass(self):
        return self.send('get_payload_mass')

    def set_payload_cog(self, x = 0, y = 0, z = 0):
        if type(x) is tuple or type(x) is list:
            y = x[1]
            z = x[2]
            x = x[0]
        return self.send('set_gravity_cog', [{'x': x, 'y': y, 'z': z}])

    def get_payload_cog(self):
        return self.send('get_payload_cog')

    def set_tcp(self, x = 0, y = 0, z = 0, rz = 0, ry = 0, rx = 0):
        return self.send('set_tcp', [[x, y, z, rz, ry, rx]])

    def get_tcp(self):
        return self.send('get_tcp')

    def get_claw_aio(self, pin):
        return self.send('get_claw_aio', [pin])

    def set_claw_aio(self, pin, value = 0):
        return self.send('set_claw_aio', [pin, value])

    def set_claw(self, force = 0, amplitude = 0):
        return self.send('set_claw', [force, amplitude])

    def movej(self, p, a = 0, v = 0, t = 0, r = 0, is_joint=None):
        is_joint = getattr(p, 'is_joint', True)
        if 'pos' not in p:
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
        return self.send('movej', [p, a, v, t, r])

    def movel(self, p: object, a = 0, v = 0, t = 0, r = 0, is_joint: bool = None):
        if type(p) is not CartesianPose and type(p) is not JointPose:
            raise Exception("请传入 CartesianPose 或 JointPose")
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
        return self.send('MoveL')

    def movec(self, via: object, p: object, rad = 0, a = 0, v = 0, t = 0, r = 0,
              is_joint: bool = None):
        if type(p) is not CartesianPose and type(p) is not JointPose:
            raise Exception("p参数必须是 CartesianPose 或 JointPose 类型")
        if type(via) is not CartesianPose and type(via) is not JointPose:
            raise Exception("via 参数 必须是 CartesianPose 或 JointPose类型")
        if not hasattr(p, 'pos'):
            p = CartesianPose(*p)
        if not hasattr(via, 'pos'):
            via = CartesianPose(*via)
        req = rc.MoveCRequest(
            pose_via=list(via.pos),
            pose_via_is_joint=getattr(via, 'is_joint', True),
            pose_to=list(p.pos),
            pose_to_is_joint=getattr(p, 'is_joint', True),
            acceleration=a,
            velocity=v,
            time=t,
            blend_radius=r,
            rad=rad
        )
        if type(p) is CartesianPose:
            p._base_set_PR(req.pose_base)
        return self.send('MoveC')

    def stop_move(self):
        """
        停止当前移动

        """
        return self.send('StopMove')

    def move_pvat(self, p: list, v: list, a: list, t: float):
        """
        指定位置、速度、加速度、时间的伺服移动

        :param p: 关节位置列表 (rad)，类型：list[float]
        :param v: 关节速度列表 (rad/s)，类型：list[float]
        :param a: 关节加速度列表 (rad/s^2)，类型：list[float]
        :param t: 总运动时间 (s)
        """
        return self.send('MovePVAT')

    def move_pvats(self, pvt_iter: list):
        """
        move_pvat的流版本

        @param pvt_iter: 类型：list[PVAT]
        @type pvt_iter: list[PVAT]
        """
        return self.send('MovePVATStream')
            # (rc.PVATRequest(duration=s.duration, q=s.q, v=s.v, acc=s.acc) for s in pvt_iter))

    def move_pvt(self, p: list, v: list, t: float):
        """
        指定位置、速度、时间的伺服移动, 加速度将自动计算。

        :param p: 关节位置列表 (rad)，类型：list[float],
        :param v: 关节速度列表 (rad/s)，类型：list[float],
        :param t: 总运动时间 (s)
        """
        return self.send('MovePVT')

    def move_pvts(self, pvt_iter: list):
        """
        指定位置、速度、时间的伺服移动, 加速度将自动计算。

        :param pvt_iter: 类型：list[PVAT]
        :return:
        """
        return self.send('MovePVTStream')
            # (rc.PVATRequest(duration=s.duration, q=s.q, v=s.v) for s in pvt_iter))

    def move_pt(self, p: list, t: float):
        """
        指定位置和时间的伺服移动,速度和加速度将自动计算。

        :param p: 关节位置列表 (rad)，类型：list[float]
        :param t: 总运动时间 (s)
        """
        return self.send('MovePT')

    def move_pts(self, pt_iter: list):
        """
        指定位置和时间的伺服移动,速度和加速度将自动计算。

        :param pt_iter: 类型：list[PVAT]
        :return:
        """
        return self.send('MovePTStream')
            # (rc.PVATRequest(duration=s.duration, q=s.q) for s in pt_iter))

    def movej_until(self, p, a=0, v=0, t=0, cb=None):
        pass

    def movej_until_rt(self, p, a=0, v=0, t=0, logic='AND', io={}, cb=None):
        pass

    def movel_until(self, p, a=0, v=0, t=0, cb=None):
        pass

    def movel_until_rt(self, p, a=0, v=0, t=0, logic='AND', io={}, cb=None):
        pass

    def movec_until(self, via, p, rad=0, a=0, v=0, t=0, cb=None):
        pass

    def movec_until_rt(self, via, p, rad=0, a=0, v=0, t=0, logic='AND', io={}, cb=None):
        pass

    def kinematics_forward(self, *p: list):
        """
        机器人正解

        :param p: 关节位置，类型：list[float]
        :return: 空间位置

        示例：

        >>> certesianPose = self.kinematics_forward(JointPose(0, -0.5, math.pi / 6, 0, 0, 0))
        """
        j = JointPose(*p)
        return self.send('KinematicsForward')

    def kinematics_inverse(self, *p: list):
        j = CartesianPose(*p)
        return self.send('KinematicsInverse')

    def pose_times(self):
        pass

    def pose_inverse(self):
        pass

    def get_actual_joint_positions(self):
        """
        获取实际关节位置

        :returns: 关节位置
        """
        return self.send('GetActualJointPositions')

    def get_target_joint_positions(self):
        """
        获得所有关节的期望角度位置

        :return: 所有关节的期望角度位置
        """
        return self.send('GetTargetJointPositions')

    def get_actual_joint_speeds(self):
        """
        获得所有关节的实际角速度

        :return: 所有关节的实际角速度
        """
        return self.send('GetActualJointSpeeds')

    def get_target_joint_speeds(self):
        """
        获得所有关节的期望角速度

        :return: 所有关节的期望角速度
        """
        return self.send('GetTargetJointSpeeds')

    def get_joint_torques(self):
        """
        获得每个关节的扭矩值

        :return: 每个关节的扭矩值
        """
        return self.send('GetJointTorques')

    def get_actual_joint_torques(self):
        """
        获取实际力矩

        :return: 实际力矩
        """
        return self.send('GetRobotData')

    def get_target_joint_torques(self):
        """
        获取理论力矩

        :return: 理论力矩
        """
        return self.send('GetRobotData')

    def get_joint_temperatures(self):
        """
        获取关节温度

        :return: 关节温度
        """
        return self.send('GetRobotData')

    def get_joint_temp(self, joint):
        """
        获取关节温度

        :param joint: 关节序号

        :returns: 关节当前温度 (℃)
        """
        return self.send('GetJointTemp')

    def get_actual_tcp_pose(self):
        """
        获取实际空间位置

        :returns: 空间位置
        """
        return self.send('GetActualTcpPose')

    def get_target_tcp_pose(self):
        """
        获得TCP的期望的姿态/位置

        :return: TCP的期望的姿态/位置
        """
        return self.send('GetTargetTcpPose')

    def get_robot_poses(self):
        """
        获取机器人姿态信息

        :return: 机器人姿态信息
        """
        return self.send('GetRobotData')

    def get_robot_data(self):
        """
        获取机器人数据

        :return: 机器人数据
        """
        return self.send('get_robot_data')

    def _generate_robot_data_cmd(self, n=1):
        for i in range(0, n):
            yield rc.RobotDataCmd()

    def get_robot_io_data(self):
        """
        获取机器人IO数据

        :return: 机器人IO数据
        """
        return self.send('GetRobotIOData')

    def set_do(self, pin, value):
        """
        设置数字输出

        :param pin: 针脚
        :param value: 值
        """
        return self.send('SetDIO')

    def get_di(self, pin):
        """
        获取数字输入

        :param pin: 针脚
        :return: 数字输入值
        """
        return self.send('GetDIO')

    def set_ao(self, pin, value: float):
        """
        设置模拟输出

        :param pin: 针脚
        :param value: 值
        """
        return self.send('SetAIO')

    def get_ai(self, pin):
        """
        获取模拟输入

        :param pin: 针脚
        :return: 模拟输入值
        """
        return self.send('GetAIO')

    def set_extra_do(self, pin, value):
        """
        设置扩展数字输出

        :param pin: 针脚
        :param value: 值
        """
        return self.send('SetExtraDIO')

    def get_extra_di(self, pin):
        """
        获取扩展数字输入

        :param pin: 针脚
        :return: 数字输入值
        """
        return self.send('GetExtraDIO')

    def set_extra_ao(self, pin, value: float):
        """
        设置扩展模拟输出

        :param pin: 针脚
        :param value: 值
        """
        return self.send('SetExtraAIO')

    def get_extra_ai(self, pin):
        """
        获取扩展模拟输入

        :param pin: 针脚
        :return: 模拟输入值
        """
        return self.send('GetExtraAIO')

    def set_ai_mode(self, pin, mode):
        """
        设置模拟输入端口工作模式

        :param pin: 针脚
        :param mode:  0:电压，1:电流
        """
        return self.send('SetAInMode')

    def get_ai_mode(self, pin):
        """
        获取模拟输入端口工作模式

        :param pin: 针脚
        """
        return self.send('GetAInMode')

    def set_ao_mode(self, pin, mode):
        """
        设置模拟输出端口工作模式

        :param pin: 针脚
        :param mode:  0:电压，1:电流
        """
        return self.send('SetAOutMode')

    def get_ao_mode(self, pin):
        """
        获取模拟输出端口工作模式

        :param pin: 针脚
        :return: 0:电压，1:电流
        """
        return self.send('GetAOutMode')

    def set_flange_do(self, pin, value):
        """
        设置法兰数字输出

        :param pin: 针脚
        :param value: 值
        """
        return self.send('SetTcpDIO')

    def get_flange_di(self, pin):
        """
        获取法兰数字输出

        :param pin: 针脚
        :return: 值
        """
        return self.send('GetTcpDIO')

    def set_led(self, mode, speed: float, color: list):
        """
        设置LED灯状态

        :param mode: 1：关闭、2：常亮、3：呼吸、4：均分旋转、5：同色旋转、6：闪烁
        :param speed: 速度 分三个等级，1：快速、2：正常、3：慢速
        :param color: 颜色，最多包含4个 0 ~ 15 之间的整数，类型：list[int]
        """
        return self.send('SetLED')

    def set_voice(self, voice, volume):
        """
        设置声音

        :voice: 声音列表 0~10
        :volume: 音量 分四个等级，0：静音、1：低、2：正常、3：高
        """
        return self.send('SetVoice')

    def set_fan(self, fan):
        """
        设置风扇

        :param fan: 1：关闭、 2：开启
        """
        return self.send('SetFan')

    def set_signal(self, pin, value):
        """
        设置信号

        :param pin: 针脚
        :param value:  值
        """
        return self.send('SetSignal')

    def get_signal(self, pin):
        """
        获取信号

        :param pin: 针脚
        """
        return self.send('GetSignal')

    def add_signal(self, pin, value):
        """
        添加信号

        :pin: 针脚
        :value: 值
        """
        return self.send('AddSignal')

    def enable_joint_limits(self):
        """
        启用关节限位检测

        """
        self.pcs.EnableJointLimit(pc.TrueOrFalse(val=True))

    def disable_joint_limits(self):
        """
        关闭关节限位检测

        """
        self.pcs.EnableJointLimit(pc.TrueOrFalse(val=False))

    def run_scene(self, scene_id, execute_count = 1, clear: bool = True):
        """
        运行场景

        :param scene_id: 场景Id
        :param execute_count: 执行数量，0 表示一直循环运行
        :param clear: 是否强制关闭正在运行的任务
        :return: 任务Id
        """
        return self.http_service.run_scene(scene_id, execute_count, clear)['id']

    def rerun_task(self, task_id, execute_count = 1, clear: bool = True):
        """
        运行任务

        :param task_id: 任务Id
        :param execute_count: 执行数量，0 表示一直循环运行
        :param clear: 是否强制关闭正在运行的任务
        :return: 任务Id
        """
        return self.http_service.run_task(task_id, execute_count, clear)['id']

    def execute_lua_code(self, task_name: str, code: str, execute_count = 1, clear: bool = True):
        """

        :param task_name: 任务名称
        :param code: lua 代码
        :param execute_count: 执行数量，0 表示一直循环运行
        :param clear: 是否强制关闭正在运行的任务
        :return: 任务Id
        """
        return self.http_service.execute_lua_code(task_name, execute_count, clear, code)['id']

    def get_task(self, id):
        """
        获取任务信息

        :param id: 任务Id
        :return: 任务信息
        """
        return self.http_service.get_task(id)

    def get_tasks(self, pi, ps):
        """
        获取任务列表

        :param pi: 页码
        :param ps: 页大小
        :return: 任务列表
        """
        return self.http_service.get_tasks(pi, ps)

    def record_pvat(self):
        """
        记录PVAT数据点
        """
        # self._sync()

        req = pc.RecordPVATRequest()
        req.type = pc.RecordPVATRequest.PVATType.PVAT
        req.duration = 200
        req.use_duration_ts = True
        req.save_file = False
        req.vZeroGap.remove_gap = False
        req.vZeroGap.threshold = 0.001

        res = self.pcs.RecordPVAT(req)
        for r in res:
            if r.end:
                return
            s = rc.PVATRequest(duration=r.t, q=r.p, v=r.v, acc=r.v)
            yield s

    def stop_record_pvat(self):
        """
        停止记录PVAT数据点
        """
        self.pcs.StopRecordPVAT(Empty())

    def get_dh_params(self, get_theoretical: bool = False):
        """
        获取当前 DH 参数
        :param get_theoretical: 是否获取理论dh参数
        """

        res = self.pcs.GetDHParams(DHRequest(get_theoretical=get_theoretical))

        for r in res.params:
            s = DHItem(r)
            yield s

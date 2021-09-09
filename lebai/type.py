from enum import Enum

from lebai.pb2 import messages_pb2 as msg
from .pb2 import robot_controller_pb2 as rc
from .pb2 import private_controller_pb2 as pc


class RobotState(Enum):
    """机器人状态"""

    DISCONNECTED = 0
    """已断开连接"""

    ESTOP = 1
    """急停停止状态"""

    BOOTING = 2
    """启动中"""

    ROBOT_OFF = 3
    """电源关闭"""
    ROBOT_ON = 4
    """电源开启"""
    IDLE = 5
    """空闲中"""
    PAUSED = 6
    """暂停中"""
    RUNNING = 7
    """机器人运动运行中"""
    UPDATING = 8
    """更新固件中"""
    STARTING = 9
    """启动中"""
    STOPPING = 10
    """停止中"""
    TEACHING = 11
    """示教中"""
    STOP = 12
    """普通停止"""
    FINETUNING = 13
    """微调中"""


class TaskStatus(Enum):
    """
    任务状态

    """

    IDLE = 0
    """空闲"""
    RUNNING = 1
    """运行中"""
    PAUSED = 2
    """暂停"""
    SUCCESS = 3
    """运行完成"""
    STOPPED = 4
    """停止"""
    ABORTED = 5
    """异常终止"""


class IODeviceType(Enum):
    """
    IO设备类型

    """
    HOST = 0
    """主机箱"""
    FLANGE = 1
    """法兰"""
    GRIPPER = 2
    """手爪"""
    MODBUS_TCP = 3
    """MODBUS/TCP"""
    # MODBUS_TCP6 = 4
    # MODBUS_RTU = 5


class CartesianPose:
    """
    空间位置

    笛卡尔坐标描述的位姿
    """

    def __init__(self, x=0, y=0, z=0, rz=0, ry=0, rx=0, base=None):
        # print(x, hasattr(x, '__iter__'), hasattr(x, '__iter__'), type(x))
        if type(x) is msg.PR:
            self.pos = [x.position.x, x.position.y, x.position.z, x.rotation.r, x.rotation.p, x.rotation.y]
        elif hasattr(x, 'pos'):
            self.pos = list(x.pos[:])
            if hasattr(x, 'base'):
                base = x.base
        elif hasattr(x, '__iter__') or type(x) is list:
            self.pos = list(x)
        else:
            self.pos = [x, y, z, rz, ry, rx]
        self.is_joint = False
        # 要求 base 是元组或列表而不是 CartesianPose 对象
        self.base = list(getattr(base, 'pos', base)) if base is not None else None

    def __str__(self):
        if self.base is None:
            return 'CartesianPose' + str(self.pos)
        else:
            return f'CartesianPose({self.pos[0]}, {self.pos[1]}, {self.pos[2]}, {self.pos[3]}, {self.pos[4]}, {self.pos[5]}, base={self.base})'

    @property
    def x(self):
        return self.pos[0]

    @property
    def y(self):
        return self.pos[1]

    @property
    def z(self):
        return self.pos[2]

    @property
    def rz(self):
        return self.pos[3]

    @property
    def ry(self):
        return self.pos[4]

    @property
    def rx(self):
        return self.pos[5]

    def __getitem__(self, item):
        if item == 'pos':
            return self.pos

    def __eq__(self, other):
        return self.pos == other.pos and self.base == other.base

    def _to_PR(self):
        p = msg.Coordinate(x=self.pos[0], y=self.pos[1], z=self.pos[2])
        r = msg.Rotation(r=self.pos[3], p=self.pos[4], y=self.pos[5])
        return msg.PR(position=p, rotation=r)

    def _base_set_PR(self, pr):
        if self.base is not None:
            pr.position.x = self.base[0]
            pr.position.y = self.base[1]
            pr.position.z = self.base[2]
            pr.rotation.r = self.base[3]
            pr.rotation.p = self.base[4]
            pr.rotation.y = self.base[5]

    def _to_Vector(self):
        return rc.Vector(vector=self.pos)


class JointPose:
    """关节位置，关节旋转角度描述的机器人姿态"""

    def __init__(self, *j):
        if len(j) > 0 and hasattr(j[0], 'pos'):
            self.pos = list(j[0].pos)
        elif len(j) > 0 and hasattr(j[0], '__iter__'):
            self.pos = list(j[0])
        else:
            self.pos = list(j)
        self.is_joint = True

    def __str__(self):
        return 'JointPose' + str(self.pos)

    def __getitem__(self, item):
        if item == 'pos':
            return self.pos

    def _to_Joint(self):
        return rc.Joint(joints=self.pos)


class Error(Exception):
    """Base class for exceptions in this module."""
    pass


class RequestError(Error):
    def __init__(self, res):
        self.code = getattr(res, 'code', -1)
        self.param = getattr(res, 'msg_params', [])
        self.data = getattr(res, 'data', None)


class TaskInfo:
    """
    任务信息

    """
    id: int
    """任务ID"""
    scene_id: int
    """场景ID"""
    execute_count: int
    """执行次数"""
    executed_count: int
    """已执行次数"""
    name: int
    """任务名"""
    status: TaskStatus
    """任务状态。1:运行中 3:运行成功 4:用户主动停止 5:异常停止"""
    comment: str
    """备注"""
    start_time: str
    """任务开始时间"""
    end_time: str
    """任务结束时间"""
    consume_time: int
    """单位秒"""
    create_time: str
    """创建时间"""
    update_time: str
    """更新时间"""

    def __init__(self, res):
        self.id = res.get("id")
        self.scene_id = res.get("scene_id")
        self.execute_count = res.get("execute_count")
        self.executed_count = res.get("executed_count")
        self.name = res.get("name")
        self.status = TaskStatus(res.get("status"))
        self.comment = res.get("comment")
        self.start_time = res.get("start_time")
        self.end_time = res.get("end_time")
        self.consume_time = res.get("consume_time")
        self.create_time = res.get("create_time")
        self.update_time = res.get("update_time")


class TasksResult:
    """
    任务列表信息

    """

    pi: int
    """页索引"""
    ps: int
    """页大小"""
    total: int
    """总数"""
    records: list
    """任务列表 TaskInfo数组"""

    def __init__(self, res):
        self.pi = res.get("pi")
        self.ps = res.get("ps")
        self.total = res.get("total")
        self.records = res.get("records")
        pass


class PVAT:
    """
    移动请求参数
    """

    duration: float
    """总运动时间"""
    q: list
    """关节位置列表 float 数组"""
    v: list
    """关节速度列表 float 数组"""
    acc: list
    """关节加速度列表 float 数组"""

    def __init__(self, duration: float, q: list, v: list, acc: list):
        """

        :param duration:
        :type duration:  float
        :param q:
        :type q: list[float]
        :param v:
        :type v: list[float]
        :param acc:
        :type acc: list[float]
        """
        self.duration = duration
        self.q = q
        self.v = v
        self.acc = acc
        pass


class RobotPoseData:
    """
    机器人姿态数据

    """

    target_joint: list
    """目标关节位置 float 数组"""
    actual_joint: list
    """实际关节位置 float 数组"""
    target_pose: list
    """目标tcp位置 float 数组"""
    actual_pose: list
    """实际tcp位置 float 数组"""

    def __init__(self, res):
        self.target_joint = list(res.targetJoint.joints)
        self.actual_joint = list(res.actualJoint.joints)
        self.target_pose = list(res.targetTcpPose.vector)
        self.actual_pose = list(res.actualTcpPose.vector)


class RobotData(RobotPoseData):
    """
    机器人数据

    """

    target_torque: list
    """理论力矩 float 数组"""
    actual_torque: list
    """实际力矩 float 数组"""
    target_vel: list
    """目标关节速度 float 数组"""
    actual_vel: list
    """实际关节速度 float 数组"""
    target_acc: list
    """加速度 float 数组"""
    actual_acc: list
    """实际加速度 float 数组"""
    temp: list
    """关节温度 float 数组"""

    def __init__(self, res):
        super().__init__(res)
        self.target_torque = list(res.targetTorque.joints)
        self.actual_torque = list(res.actualTorque.joints)
        self.target_vel = list(res.targetJointSpeed.joints)
        self.actual_vel = list(res.actualJointSpeed.joints)
        self.target_acc = list(res.targetJointAcc.joints)
        self.actual_acc = list(res.actualJointAcc.joints)
        self.temp = list(res.jointTemps.joints)


class IOItem:
    pin: int
    """针脚"""
    value: float
    """值"""

    def __init__(self, pin: int, value: float):
        self.pin = pin
        self.value = value


class RobotIOData:
    di: list
    """所有数字量的输入值 IOItem 数组"""
    do: list
    """所有数字量的输出值 IOItem 数组"""
    ai: list
    """所有模拟量输入值 IOItem 数组"""
    ao: list
    """所有模拟量输出值 IOItem 数组"""
    flange_di: list
    """所有tcp数字量的输入值 IOItem 数组"""
    flange_do: list
    """所有tcp数字量的输出值 IOItem 数组"""

    def __init__(self, io):
        self.di = list(map(lambda dio: IOItem(dio.pin, dio.value), io.robotDIOIn))
        self.do = list(map(lambda dio: IOItem(dio.pin, dio.value), io.robotDIOOut))
        self.ai = list(map(lambda aio: IOItem(aio.pin, aio.value), io.robotAIOIn))
        self.ao = list(map(lambda aio: IOItem(aio.pin, aio.value), io.robotAIOOut))
        self.flange_di = list(map(lambda dio: IOItem(dio.pin, dio.value), io.tcpDIOIn))
        self.flange_do = list(map(lambda dio: IOItem(dio.pin, dio.value), io.tcpDIOOut))

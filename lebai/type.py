from enum import Enum

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

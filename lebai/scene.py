import socket
import time

from lebai.type import TaskStatus, TaskInfo
from lebai.lebai_http_service import LebaiHttpService


class LebaiScene:

    def __init__(self, ip, scene_id=0, task_id=0):
        self.ip = ip
        self.scene_id = scene_id
        self.task_id = task_id
        self.http_service = LebaiHttpService(ip)

    def action(self, cmd: str, data: object = None, sleep: float = 0) -> object:
        """
        调用命令

        :param cmd: 机器人命令
        :param data: 机器人命令参数
        :param sleep:
        :return: 命令的返回数据
        """
        r = self.http_service.action({
            'cmd': cmd,
            'data': data
        })
        if sleep > 0:
            time.sleep(1)
        return r

    def start(self, execute_count: int = 1, clear: bool = True) -> None:
        """
        开始执行

        :param execute_count: 执行次数，0表示无限循环
        :param clear: 是否停止当前正在执行的任务
        """
        if self.task_id > 0:
            self.task_id = self.http_service.run_task(self.task_id, execute_count, clear)['id']
        else:
            self.task_id = self.http_service.run_scene(self.scene_id, execute_count, clear)['id']

    def pause(self) -> None:
        """
        暂停执行
        """
        self.action('pause_task', sleep=1)

    def resume(self) -> None:
        """
        恢复执行
        """
        self.action('resume_task', sleep=1)

    def stop(self) -> None:
        """
        停止
        """
        self.action('stop_task', sleep=1)

    def result(self) -> TaskInfo:
        """
        获取任务信息

        :return: 任务信息
        """
        return self.http_service.get_task(self.task_id)

    def status(self) -> TaskStatus:
        """
        获取任务状态

        :return: 任务状态
        """
        return TaskStatus(self.result().status)

    def done(self) -> bool:
        """
        获取是否停止了执行

        :return: 是否停止了执行
        """
        status = self.status()
        if status == TaskStatus.SUCCESS or status == TaskStatus.STOPPED or status == TaskStatus.ABORTED:
            return True
        return False

    def run(self, loop: int = 1, timeout: int = 0) -> str:
        """
        运行任务或者场景，直到运行完成，返回lua代码

        :param loop: 执行次数
        :param timeout: 指定运行超时时间，0: 永不超时  (默认)，单位：秒
        :return: 返回执行了的lua代码，没有执行lua代码则返回空
        """
        output = b''
        if timeout > 0:
            t = time.time()

        self.start(loop, True)
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(0.1)
            s.connect((self.ip, 5180))
            while True:
                if 0 < timeout < (time.time() - t):
                    raise TimeoutError
                if self.done():
                    break
                try:
                    output += s.recv(1024)
                except socket.timeout:
                    pass
                if self.done():
                    break
        return output.decode('utf-8')

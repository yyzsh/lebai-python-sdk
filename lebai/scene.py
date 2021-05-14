import json
import socket
import time

from .lebai_http_service import LebaiHttpService
from .type import *


class LebaiScene:
    def __init__(self, ip, scene_id=0, task_id=0):
        self.ip = ip
        self.scene_id = scene_id
        self.task_id = task_id
        self.http_service = LebaiHttpService(ip)

    def action(self, cmd, data=None, sleep=0):
        r = self.http_service.action({
            'cmd': cmd,
            'data': data
        })
        if sleep > 0:
            time.sleep(1)
        return r

    def start(self, loop=1, force=False):
        if self.task_id > 0:
            self.task_id = self.http_service.run_task(self.task_id, loop, force)['id']
        else:
            self.task_id = self.http_service.run_scene(self.scene_id, loop, force)['id']

    def pause(self):
        self.action('pause_task', sleep=1)

    def resume(self):
        self.action('resume_task', sleep=1)

    def stop(self):
        self.action('stop_task', sleep=1)

    def result(self):
        return self.http_service.get_task(self.task_id)

    def status(self):
        return TaskStatus(self.result()['status'])

    def done(self):
        status = self.status()
        if status == TaskStatus.SUCCESS or status == TaskStatus.STOPPED or status == TaskStatus.ABORTED:
            return True
        return False

    def run(self, loop=1):
        output = b''
        self.start(loop, True)
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((self.ip, 5180))
            while True:
                if self.done():
                    break
                output += s.recv(1024)
                if self.done():
                    break
        return output.decode('utf-8')

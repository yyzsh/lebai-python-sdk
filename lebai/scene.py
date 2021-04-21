import requests
import json
import time
import socket

from .type import *

class LebaiScene:
    def __init__(self, ip, scene_id=0, task_id=0):
        self.ip = ip
        self.scene_id = scene_id
        self.task_id = task_id
        self.robot = LebaiRobot(ip)

    def start(self, loop=1, force=False):
        payload = {
            'execute_count': loop,
            'clear': 1 if force else 0
        }
        if self.task_id > 0:
            payload['task_id'] = self.task_id
        else:
            payload['scene_id'] = self.scene_id
        payload = json.dumps(payload)
        r = requests.post("http://{0}/public/task".format(self.ip), data=payload)
        r.raise_for_status()
        r = r.json()
        if r['code'] == 0:
            self.task_id = r['data']['id']
        else:
            raise RequestError(r)

    def pause(self):
        self.robot.action('pause_task', sleep=1)

    def resume(self):
        self.robot.action('resume_task', sleep=1)

    def stop(self):
        self.robot.action('stop_task', sleep=1)
    
    def result(self):
        r = requests.get("http://{0}/public/task".format(self.ip), params={'id': self.task_id})
        r.raise_for_status()
        r = r.json()
        if r['code'] == 0:
            return r['data']
        else:
            raise RequestError(r)

    def status(self):
        return TaskStatus(self.result()['status'])

    def done(self):
        status = self.status()
        if status == TaskStatus.SUCCESS or status == TaskStatus.STOPPED or status == TaskStatus.ABORTED:
            return True
        return False

    def run(self, loop=1):
        output = b''
        self.start()
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((self.ip, 5180))
            while True:
                output += s.recv(1024)
                if self.done():
                    break
        return output.decode('utf-8')

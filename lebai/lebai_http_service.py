import json
from typing import Optional

import requests

from lebai import RequestError
from lebai.type import TaskInfo, TasksResult


class LebaiHttpService:
    def get_url(self, path):
        return "http://{0}{1}".format(self.ip, path)

    def __init__(self, ip):
        self.ip = ip
        pass

    def handle_result(self, r):
        r.raise_for_status()
        r = r.json()
        if r['code'] == 0:
            return r['data']
        else:
            raise RequestError(r)

    def action(self, cmd, inner_data=None):
        data = {
            'cmd': cmd
        }
        if inner_data is not None:
            data['data'] = inner_data
        r = requests.post(self.get_url("/public/robot/action"), data=json.dumps(data))
        return self.handle_result(r)

    def run_scene(self, scene_id, execute_count, clear):
        payload = {
            'execute_count': execute_count,
            'clear': 1 if clear else 0,
            'scene_id': scene_id
        }
        r = requests.post(self.get_url("/public/task"), data=json.dumps(payload))
        return self.handle_result(r)

    def run_task(self, task_id, execute_count, clear):
        payload = {
            'execute_count': execute_count,
            'clear': 1 if clear else 0,
            'task_id': task_id
        }
        r = requests.post(self.get_url("/public/task"), data=json.dumps(payload))
        return self.handle_result(r)

    def execute_lua_code(self, task_name, code, execute_count, clear):
        r = requests.post(self.get_url("/public/executor/lua"), params={
            'name': task_name,
            'execute_count': execute_count,
            'clear': clear
        }, data=code)
        return self.handle_result(r)

    def get_task(self, id) -> Optional[TaskInfo]:
        r = requests.get(self.get_url("/public/task"), params={
            'id': str(id)
        })
        r.raise_for_status()
        r = r.json()
        if r['code'] == 0:
            return TaskInfo(r['data'])
        else:
            return None

    def get_tasks(self, pi, ps) -> Optional[TasksResult]:
        params = json.dumps({
            'pi': pi,
            'ps': ps
        })
        r = requests.get(self.get_url("/public/tasks"), params)
        r.raise_for_status()
        r = r.json()
        if r['code'] == 0:
            return TasksResult(r['data'])
        else:
            return None

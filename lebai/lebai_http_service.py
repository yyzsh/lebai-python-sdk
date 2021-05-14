import json

import requests

from lebai import RequestError


class LebaiHttpService:

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
        r = requests.post("http://{0}/public/robot/action".format(self.ip), data=json.dumps(data))
        return self.handle_result(r)

    def run_scene(self, scene_id, execute_count, clear):
        payload = {
            'execute_count': execute_count,
            'clear': 1 if clear else 0,
            'scene_id': scene_id
        }
        r = requests.post("http://{0}/public/task".format(self.ip), data=json.dumps(payload))
        return self.handle_result(r)

    def run_task(self, task_id, execute_count, clear):
        payload = {
            'execute_count': execute_count,
            'clear': 1 if clear else 0,
            'task_id': task_id
        }
        r = requests.post("http://{0}/public/task".format(self.ip), data=json.dumps(payload))
        return self.handle_result(r)

    def execute_lua_code(self, task_name, execute_count, clear, code):
        r = requests.post("http://{0}/public/executor/lua".format(self.ip), params={
            'name': task_name,
            'execute_count': execute_count,
            'clear': clear
        }, data=code)
        return self.handle_result(r)

    def get_task(self, id):
        r = requests.get("http://{0}/public/task".format(self.ip), params={
            'id': str(id)
        })
        r.raise_for_status()
        r = r.json()
        if r['code'] == 0:
            return r['data']
        else:
            return None

    def get_tasks(self, pi, ps):
        params = json.dumps({
            'pi': pi,
            'ps': ps
        })
        r = requests.get("http://{0}/public/tasks".format(self.ip), params)
        r.raise_for_status()
        r = r.json()
        if r['code'] == 0:
            return r['data']
        else:
            return None

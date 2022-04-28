import json
import logging
import websocket
import time


class JsonRpcClient:
    def __init__(self, ip, port=3031, debug=False):
        self.id = 1
        self.ip = ip
        self.port = port
        self._debug = debug
        self._res = {}

        addr = "ws://{}:{}".format(ip, port)
        self.ws = websocket.WebSocket()
        self.ws.connect(addr)

    def close(self):
        self.ws.close()

    def _parse_res(self, res):
        if 'result' in res:
            return res['result']
        elif 'error' in res:
            raise Exception(res['error'])
        else:
            raise Exception(res)

    def recv(self, id=None):
        line = self.ws.recv()
        if self._debug:
            logging.debug(line)
        res = json.loads(line)
        if 'id' in res:
            if id is None or res['id'] == id:
                return self._parse_res(res)
            else:
                if self._res.has_key(id):
                    return self._res[id]
                else:
                    self._res[res['id']] = res
                    while not self._res.has_key(id):
                        time.sleep(0.01)
                    res = self._res[id]
                    return self._parse_res(res)
        else:
            return self._parse_res(res)

    def send(self, cmd, data=None, sync=True):
        id = self.id + 1
        line = json.dumps({
            "jsonrpc": "2.0",
            "method": cmd,
            "params": data,
            "id": id
        })
        self.id = id
        self.ws.send(line)
        if self._debug:
            logging.debug(line)
        if sync:
            return self.recv(id)

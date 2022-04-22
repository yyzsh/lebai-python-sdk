import json
import websocket

class JsonRpcClient:
    def __init__(self, ip, port = 3031):
        self.id = 1
        self.ip = ip

        addr = "ws://{}:{}".format(ip, port)
        self.ws = websocket.WebSocket()
        self.ws.connect(addr)

    def close(self):
        self.ws.close()

    def send(self, cmd, data=None):
        self.id += 1
        line = json.dumps({"jsonrpc": "2.0", "method": cmd, "params": data, "id": self.id})
        self.ws.send(line)
        res = json.loads(self.ws.recv())
        if 'result' in res and 'id' in res:
            if res['id'] == self.id:
                return res['result']
            else:
                raise Exception(res['result'])
        elif 'error' in res:
            raise Exception(res['error'])
        else:
            raise Exception(res)

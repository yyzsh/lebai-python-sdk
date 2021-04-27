# 乐白机器人 Python SDK

## 使用说明

```
pip install lebai
```

[API 文档](http://lebai.py.kingfree.moe)

- `LebaiRobot`：提供同步的 API。

安装依赖（Python 3.5+）：
```bash
pip install grpcio protobuf
```

示例：

```python
import math

from lebai import LebaiRobot, CartesianPose, JointPose

def run():
    rb = LebaiRobot("192.168.3.218")

    rb.start_sys()

    rb.movej(JointPose(0, -1.2, math.pi/6, 0, math.pi/4, 0), 0, 0, 1, 0)

    base = rb.get_actual_tcp_pose()
    p2 = CartesianPose(0.1, 0, 0, 0, 0, 0, base=base)
    rb.movel(p2, 0, 0, 1, 0)

    rb.stop_sys()

if __name__ == '__main__':
    run()
```

### 安装 Python 和 pip

查看 Python 版本：
```bash
python --version
```

如果 Python 版本小于 3.7：
```bash
sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt update
sudo apt install python3.9 python3.9-distutils
```

Ubunutu 16.04
```bash
sudo apt remove python3-pip
sudo python3.9 -m easy_install pip
```

如果 Python 环境配置有问题：
```bash
sudo ln -sf /usr/bin/python3.9 /usr/bin/python3
sudo ln -sf /usr/bin/python3 /usr/bin/python
sudo ln -sf /usr/bin/pip3 /usr/bin/pip
python -m pip install --upgrade pip setuptools wheel
```

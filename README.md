# 乐白机器人 Python SDK

## 使用说明

```
pip install lebai
```

[API 文档](http://lebai.py.kingfree.moe)

- `LebaiRobot`：提供同步的 API。


## 示例

```python
import math

from lebai import LebaiRobot, CartesianPose, JointPose


def run():
    rb = LebaiRobot("192.168.3.218")

    rb.start_sys()

    rb.movej(JointPose(0, -1.2, math.pi / 6, 0, math.pi / 4, 0), 0, 0, 1, 0)

    base = rb.get_actual_tcp_pose()
    p2 = CartesianPose(0.1, 0, 0, 0, 0, 0, base=base)
    rb.movel(p2, 0, 0, 1, 0)

    rb.stop_sys()


if __name__ == '__main__':
    run()
```

## 版本要求

- Python >= 3.5

安装依赖：
```bash
pip install grpcio protobuf requests
```

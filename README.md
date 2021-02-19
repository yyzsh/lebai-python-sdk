# 乐白机器人 Python SDK

## 安装

### 安装 Python 和 pip

例如在 Ubuntu 下：
```bash
sudo apt install python3 python3-pip
```

如果 Python 环境配置有问题：
```bash
sudo ln -s /usr/bin/python3 /usr/bin/python
sudo ln -s /usr/bin/pip3 /usr/bin/pip
python -m pip install --upgrade pip setuptools wheel
```

- [https://pip.pypa.io/en/stable/installing/]()

### 安装项目依赖

安装依赖：
```
pip install -r requirements.txt
```

- [https://grpc.io/docs/languages/python/quickstart/]()
- [https://setuptools.readthedocs.io/en/latest/userguide/quickstart.html]()


## 运行

```bash
GRPC_TRACE=all GRPC_VERBOSITY=debug ./main.py
```

## 构建和发布

```bash
python3 -m pip install --user --upgrade setuptools wheel twine
python3 setup.py sdist bdist_wheel
python3 -m twine upload --repository testpypi dist/*

python3 -m twine upload --repository pypi dist/*
```

## 从 PyPI 安装

```bash
python3 -m pip install --index-url https://test.pypi.org/simple/ --no-deps lebai

pip install lebai
```

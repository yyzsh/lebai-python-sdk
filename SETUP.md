## 开发人员指南

- [https://pip.pypa.io/en/stable/installing/]()
- [https://packaging.python.org/tutorials/managing-dependencies/]()

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
python -m pip install --user pipenv
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

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

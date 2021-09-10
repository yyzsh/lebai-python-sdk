#!/bin/bash
# python -m grpc_tools.protoc -I./protos --python_out=./lebai/pb2 --grpc_python_out=./lebai/pb2 ./protos/*.proto
rm -rf dist/ lebai.egg-info/
 make html
# scp -r build/html/* root@toyama.moe:/var/www/lebai.py/

python3 setup.py sdist bdist_wheel

python3 -m twine upload --repository pypi dist/*

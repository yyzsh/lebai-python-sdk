#!/bin/bash
python -m grpc_tools.protoc -I./protos --python_out=./lebai/pb2 --grpc_python_out=./lebai/pb2 ./protos/*.proto

make html

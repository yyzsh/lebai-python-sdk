#!/usr/bin/python3
# -*- coding: utf-8 -*-

from lebai import rc, robot_controller_pb2_grpc
import numpy as np
import asyncio
import grpc

def generate_list(rcs, filename):
    points = np.loadtxt(filename, delimiter=',')
    rcs.MoveJ(rc.MoveJRequest(
        joint_pose_to = points[0],
        pose_is_joint_angle=True,
        acceleration=1,
        velocity=1,
        time=0,
        blend_radius=0
    ))
    # print(np.shape(points))
    vels = np.empty(np.shape(points))
    vels[0, :] = [0,0,0,0,0,0]
    n = range(np.shape(points)[0]-1)
    for i in n:
        vels[i+1, :] = (points[i, :] - points[i+1, :])/0.01
        data = rc.PVATRequest(
            duration=0.01,
            q=points[i],
            v=vels[i]
        )
        print(data)
        yield data

if __name__ == '__main__':
    rcc = grpc.insecure_channel('192.168.3.218:5181')
    rcs = robot_controller_pb2_grpc.RobotControllerStub(rcc)
    
    i = generate_list(rcs, 'pose.txt')
    rcs.MovePVTStream(i)

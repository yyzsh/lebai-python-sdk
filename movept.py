#!/usr/bin/python3
# -*- coding: utf-8 -*-

from lebai import LebaiRobot
import numpy as np
import grpc

def generate_list(rcs, filename):
    points = np.loadtxt(filename, delimiter=',')
    rc.movej(points[0], 1, 1)
    # print(np.shape(points))
    vels = np.empty(np.shape(points))
    vels[0, :] = [0,0,0,0,0,0]
    n = range(np.shape(points)[0]-1)
    t = 0.01
    for i in n:
        vels[i+1, :] = (points[i, :] - points[i+1, :])/t
        data = {
            "t": t,
            "p": points[i],
            "v": vels[i]
        }
        print(data)
        yield data

if __name__ == '__main__':
    rc = LebaiRobot('192.168.3.218')
    
    i = generate_list(rc, 'examples/pose.txt')
    rc.move_pvts(i)

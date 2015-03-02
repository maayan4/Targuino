# -*- coding: utf-8 -*-
"""
Created on Sat Feb 21 22:27:06 2015

@author: maayan4
"""

import numpy as np, scipy as sp, matplotlib as mpl

fileName='/home/maayan4/Reps/Targuino/log.dat'

num_lines = sum(1 for line in open(fileName))

arr=np.zeros([1,num_lines])
num=0

with open(fileName) as fid:
    for line in fid:
        tmp=line[0:4]
        if tmp.isdigit():
            arr[0,num]=int(line[0:4])
            num=num+1


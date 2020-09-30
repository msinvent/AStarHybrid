#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Sep 28 21:39:21 2020

@author: manish
"""


import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
import time

plt.close(0)

df = pd.read_csv('path.txt')
plt.figure(0);
# plt.  .hold(True)
val = max(max(df['y']),max(df['y'])) + 2
plt.axis([-1,val,-1,val])
plt.plot(df['x'], df['y'],'k-')
plt.plot(np.arange(val),'g-')


df2 = pd.read_csv('exploredGraph.txt')
plt.plot(df2['x'], df2['y'],'r*')
plt.grid()
plt.show()

# for x in range(0, len(df['x'])):
#     print(x)
#     plt.figure(1);
#     plt.axis([0,20,0,20])
#     plt.plot(df2['x'][x], df2['y'][x],'r*')
#     plt.show()
#     time.sleep(1)
#     plt.close(1)
# plt.plot(np.arange(50))
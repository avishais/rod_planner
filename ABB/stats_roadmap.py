import numpy as np
import os
import subprocess


# import sys,os
# sys.path.append('/home/avishai/Documents/workspace/rod_planner/roadmap/')

for _ in range(10):
    subprocess.call(['../roadmap/gen', '100 4 1'])
    os.system('./plan')

    A = np.loadtxt('./path/afile.txt')
    Q = np.loadtxt('./path/robot_paths.txt', skiprows=1)

    la = np.sum([np.linalg.norm(A[i]-A[i-1]) for i in range(1, A.shape[0])])
    lq = np.sum([np.linalg.norm(Q[i]-Q[i-1]) for i in range(1, Q.shape[0])])

    ft = open('./stats/stats_rm.txt', 'a')
    ft.write('%f %f\n'%(la,lq))
    ft.close()


import numpy
import os
import subprocess


# import sys,os
# sys.path.append('/home/avishai/Documents/workspace/rod_planner/roadmap/')

for _ in range(7):
    subprocess.call(['../roadmap/gen', '5 4 1'])
    # os.system('./plan')
    os.system('../ABB/plan')
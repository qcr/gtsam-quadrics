import os
import sys
sys.path.append('/home/lachness/.pyenv/versions/382_generic/lib/python3.8/site-packages/')
sys.path.append('/home/lachness/git_ws/quadricslam/ros/src/py_detector/py_detector')

import numpy as np
import gtsam
import quadricslam


def main():
    print(quadricslam.AlignedBox2(1,2,3,4))

if __name__ == '__main__':
    main()
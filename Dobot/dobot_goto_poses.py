#!/usr/bin/env python
# -*- coding:utf-8 -*-

"""
该程序读取指定的位姿文件，并依次移动到这些点位

作者：仝铁
创建日期：2025年3月27日

"""

import os
import time
import sys

sys.path.append('/home/eaibot/scripts')

from Dobot_Api import f_GetPose, f_SetPTPCmd
from utils import read_pose_file    # 导入自定义的工具函数


os.chdir('/home/eaibot/scripts/dobot_poses')


def move_to_poses(poses):
    """
    依次移动到指定的点位
    
    参数:
    - poses: 位姿列表
    """
    print("开始移动到预设点位")
    
    for i, pose in enumerate(poses, 1):
        # 打印当前点位信息
        print("\n移动到第 {0} 个点位:".format(i))
        print("坐标 - X: {0}, Y: {1}, Z: {2}, R: {3}".format(
            pose['x'], pose['y'], pose['z'], pose['r']
        ))
        
        # 调用 PTP 移动命令
        try:
            result = f_SetPTPCmd(
                float(pose['x']), 
                float(pose['y']), 
                float(pose['z']), 
                float(pose['r'])
            )
            print("移动结果: {0}".format(result))
        except Exception as e:
            print("移动时发生错误: {0}".format(e))
        
        time.sleep(3)  # 等待 3 秒
        

def main():
    # 选择位姿文件
    filename = "poses.csv"
    
    if filename is None:
        print("无该文件，程序退出")
        return
    
    # 读取位姿
    poses = read_pose_file(filename)
    
    if poses:
        # 开始移动到点位
        move_to_poses(poses)


if __name__ == "__main__":
    main()
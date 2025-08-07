#!/usr/bin/env python
# -*- coding:utf-8 -*-

"""
功能：键盘交互式读取指定的位姿文件，并依次移动到这些点位

作者：仝铁
创建日期：2025年3月27日
修改日期：2025年3月28日

"""

import os
import sys

sys.path.append('/home/eaibot/scripts')

from Dobot_Api import f_GetPose, f_SetPTPCmd
from utils import getch, read_pose_file    # 导入自定义的工具函数


os.chdir('/home/eaibot/scripts')


def select_pose_file():
    """
    选择要读取的位姿文件
    
    返回:
    - 选中的文件路径，如果取消则返回 None
    """
    # 获取当前目录下的所有 CSV 文件
    csv_files = [f for f in os.listdir('dobot_poses') if f.endswith('.csv') and f.startswith('dobot_poses_')]
    
    if not csv_files:
        print("没有找到位姿文件")
        return None
    
    print("找到以下位姿文件：")
    for i, file in enumerate(csv_files, 1):
        print("{0}. {1}".format(i, file))
    
    while True:
        try:
            print("\n请选择文件（输入序号）或按 'q' 退出：")
            choice = getch()
            
            if choice == 'q':
                return None
            
            index = int(choice)
            if 1 <= index <= len(csv_files):
                return csv_files[index-1]
            else:
                print("无效的选择，请重新输入")
        
        except ValueError:
            print("请输入有效的数字")
        except IndexError:
            print("请输入有效的序号")


def move_to_poses(poses):
    """
    依次移动到指定的点位
    
    参数:
    - poses: 位姿列表
    """
    print("开始移动到预设点位")
    print("操作说明:")
    print("  - 按任意键继续移动到下一个点")
    print("  - 按 'q' 键随时退出")
    
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
        
        # 等待用户确认继续
        print("按任意键继续，按 'q' 退出...")
        key = getch()
        
        if key == 'q':
            print("已中断点位移动")
            break


def main():
    while True:
        # 选择位姿文件
        filename = select_pose_file()
        
        if filename is None:
            print("程序退出")
            break
        
        # 读取位姿
        poses = read_pose_file(filename)
        
        if poses:
            # 开始移动到点位
            move_to_poses(poses)
        
        # 询问是否继续
        print("\n是否继续操作？(y/n)")
        choice = getch()
        if choice.lower() != 'y':
            break

if __name__ == "__main__":
    main()
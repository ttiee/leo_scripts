#!/usr/bin/env python
# -*- coding:utf-8 -*-

"""
该文件包含一个交互式的位姿采集程序，用于采集机械臂的一系列位姿信息并保存到CSV文件中
使用按键 'p' 采集当前位姿，按键 'q' 保存并退出，按键 'c' 取消并退出

作者：仝铁
创建日期：2025年3月27日
修改日期：2025年3月28日

"""

import csv
import datetime
import os
import sys

sys.path.append('/home/eaibot/scripts')

from Dobot_Api import f_GetPose
from utils.getch import getch
from utils.parse_data import parse_pose_data

os.chdir('/home/eaibot/scripts')



def collect_poses():
    """
    交互式收集机械臂位姿
    
    返回:
    - 保存的位姿列表
    """
    # 生成唯一的文件名，包含当前时间戳
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = "dobot_poses/dobot_poses_{0}.csv".format(timestamp)
    
    poses = []
    
    while True:
        print("Dobot位姿采集程序")
        print("操作说明:")
        print("  - 按 'p' 键获取并保存当前位姿")
        print("  - 按 'q' 键退出并保存文件")
        print("  - 按 'c' 键取消并退出（不保存）")
        print("\n等待指令... ")
        key = getch()
        
        if key == 'p':
            # 获取并保存位姿
            try:
                # 获取当前位姿文本
                pose_text = f_GetPose()
                
                # 解析位姿数据
                pose_data = parse_pose_data(pose_text)
                
                # 添加序号和时间戳
                pose_entry = {
                    'index': len(poses) + 1,
                    'timestamp': datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                    'result': pose_data['result'],
                    'x': pose_data['x'],
                    'y': pose_data['y'],
                    'z': pose_data['z'],
                    'r': pose_data['r'],
                    'jointAngle': str(pose_data['jointAngle'])  # 转换为字符串以便CSV存储
                }
                
                poses.append(pose_entry)
                print("位姿已保存 - 第 {0} 个点: {1}".format(len(poses), pose_entry))
            
            except Exception as e:
                print("获取位姿时发生错误: {0}".format(e))
        
        elif key == 'q':
            # 保存并退出
            if poses:
                try:
                    with open(filename, 'wb') as csvfile:
                        fieldnames = poses[0].keys()
                        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                        
                        # 写入表头
                        writer.writeheader()
                        
                        # 写入数据
                        for pose in poses:
                            writer.writerow(pose)
                    
                    print("位姿信息已保存到 {0}".format(filename))
                    break
                
                except IOError as e:
                    print("保存文件时发生错误: {0}".format(e))
            else:
                print("没有采集到任何位姿，直接退出")
                break
        
        elif key == 'c':
            # 取消并退出
            print("已取消位姿采集")
            return None
        
        else:
            print("无效的按键，请重新输入")
    
    return poses

def main():
    # 开始交互式位姿采集
    collect_poses()



if __name__ == "__main__":
    main()
#!/usr/bin/env python
# -*- coding:utf-8 -*-

"""
该文件包含一个交互式的位姿采集程序，用于采集机械臂的一系列位姿信息并保存到CSV文件中
使用按键 'p' 采集当前位姿，按键 'q' 保存并退出，按键 'c' 取消并退出，按键 't' 启动取放盒子功能
使用按键 's' 标记在当前位姿开启气泵
功能：读取csv位姿文件，依次到达一系列点位，并在特殊点位处取盒子，然后放置到指定位置

作者：human-made Fe
创建日期：2025年5月13日
修改日期：2025年5月17日

"""


import csv
import datetime
import os
import sys
import time

sys.path.append('/home/eaibot/scripts')

from Dobot_Api import f_GetPose, f_SetPTPCmd, f_SetEndEffectorSuctionCup
from utils.getch import getch
from utils.parse_data import parse_pose_data
from utils import read_pose_file  # 导入读取位姿文件功能

os.chdir('/home/eaibot/scripts')


def move_to_poses(poses):
    """
    依次移动到指定的点位，并在标记的点位处开启/关闭气泵
    
    Args:
        poses: 位姿点位列表
        detect_province: 是否检测省份(默认为False，本函数不执行省份检测)
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

        # 检查是否需要开启气泵
        if 'suction_on' in pose and pose['suction_on'] == 'True':
            print("在第{0}个点位开启气泵".format(i))
            f_SetEndEffectorSuctionCup(True)
            time.sleep(1)  # 等待气泵稳定
        
        # 如果是最后一个点位，则关闭气泵
        if i == len(poses):
            print("在最后一个点位关闭气泵")
            f_SetEndEffectorSuctionCup(False)
            time.sleep(1)  # 等待1秒
            print("已到达最后一个点位")
            break
    
    print("点位序列复现完成")
    return None


def collect_poses():
    """
    交互式收集机械臂位姿
    
    返回:
    - 保存的位姿列表
    """
    filename = "dobot_poses/dobot_poses_show.csv"
    
    poses = []
    
    while True:
        print("Dobot位姿采集程序")
        print("操作说明:")
        print("  - 按 'p' 键获取并保存当前位姿")
        print("  - 按 's' 键获取当前位姿并标记为开启气泵点位")
        print("  - 按 'q' 键退出并保存文件")
        print("  - 按 'c' 键取消并退出（不保存）")
        print("  - 按 't' 键启动取放盒子功能")
        print("\n等待指令... ")
        key = getch()
        
        if key == 'p' or key == 's':
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
                    'jointAngle': str(pose_data['jointAngle']),  # 转换为字符串以便CSV存储
                    'suction_on': 'True' if key == 's' else 'False'  # 标记是否开启气泵
                }
                
                poses.append(pose_entry)
                
                # 根据不同键显示不同信息
                if key == 's':
                    print("位姿已保存 - 第 {0} 个点: {1} (开启气泵)".format(len(poses), pose_entry))
                else:
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
            
        elif key == 't':
            # 启动取放盒子功能
            print("启动取放盒子功能...")
            try:
                # 读取位姿文件
                csv_path = filename
                if not os.path.exists(csv_path):
                    print("位姿文件不存在，请先采集位姿")
                    continue
                # 读取位姿数据
                poses_data = read_pose_file(csv_path)
                
                if poses_data:
                    # 执行移动到点位序列
                    move_to_poses(poses_data)
                    print("盒子取放完成")
                else:
                    print("未找到有效的位姿数据，请先保存位姿")
            except Exception as e:
                print("执行取放盒子功能时出错: {0}".format(e))
        
        else:
            print("无效的按键，请重新输入")
    
    return poses


def main():
    # 开始交互式位姿采集
    collect_poses()


if __name__ == "__main__":
    main()
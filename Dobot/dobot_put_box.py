#!/usr/bin/env python
# -*- coding:utf-8 -*-

"""
功能：读取csv位姿文件，依次到达一系列点位，并在特殊点位处取盒子，然后放置到指定位置

作者：One Boat
创建日期：2025年3月27日
修改日期：2025年3月30日

"""

import time
import sys

sys.path.append('/home/eaibot/scripts')

from Dobot_Api import f_SetPTPCmd, f_SetEndEffectorSuctionCup
from utils import read_pose_file    # 读取csv位姿文件


# csv文件的位置
csv_file_name = '/home/eaibot/scripts/dobot_poses/poses_put.csv'


def move_to_poses(poses):
    """
    依次移动到指定的点位，并在特定点位处取盒子"
    """
    print("开始移动到预设点位")
    province_detected = None
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

        # 如果是第三个点位，则取盒子
        if i == 2:
            print("在第4个点位取盒子")
            f_SetEndEffectorSuctionCup(True)
            time.sleep(1)  # 等待1秒

        # 如果是第六个点位，则放置盒子
        if i == 4:
            print("在第9个点位放置盒子")
            if province_detected:
                print("将盒子放置在{0}对应的位置".format(province_detected))
            f_SetEndEffectorSuctionCup(False)
            time.sleep(1)  # 等待1秒
        
        # 如果是最后一个点位，则结束
        if i == len(poses):
            print("已到达最后一个点位")
            break
    return province_detected

def main():
    # 选择位姿文件
    filename = csv_file_name

    if filename is None:
        print("无该文件：{0}".format(filename))
        return

    # 读取位姿
    poses = read_pose_file(filename)

    if poses:
        # 开始移动到点位
        move_to_poses(poses)


if __name__ == '__main__':
    main()
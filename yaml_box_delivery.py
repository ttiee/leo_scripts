#!/usr/bin/env python
# -*- coding:utf-8 -*-

"""
功能：读取yaml文件中的位置信息，从起始点出发通过导航到达货架1，
     取盒子后，获取省份信息并导航到对应省份点位

作者：张一舟
创建日期：2025年3月28日
修改日期：2025年3月30日

"""

import rospy
import time
import os
import sys
import threading

from Nav_Api import Get_current_pose, navigate_to_position, set_xy_goal_tolerance, set_yaw_goal_tolerance
from Dobot.dobot_take_box import main as take_box_main
from Dobot.dobot_put_box import main as put_box_main
from utils import read_yaml_file
from robot_correct_position import correct_robot_position_by_pose, correct_robot_position   # 导入位置修正函数

# 设置默认编码为UTF-8（仅在Python 2中有效）
if sys.version_info[0] < 3:
    reload(sys)
    sys.setdefaultencoding('utf-8')


# yaml文件的位置
yaml_position_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'robot_position/positions.yaml')


def run_position_correction():
    """
    在后台线程中运行自动位置修正功能
    """
    print("启动自动位置修正线程...")
    while not rospy.is_shutdown():
        time.sleep(5)  # 每5秒检查一次位置
        # 调用位置修正函数
        correct_robot_position()


def main():
    # 初始化ROS节点
    try:
        if not rospy.core.is_initialized():
            rospy.init_node('delivery_robot', anonymous=True)
            print("ROS节点初始化成功")
    except rospy.exceptions.ROSException as e:
        print("ROS节点已经初始化: {}".format(e))
        # 继续执行，因为节点已经初始化

    # 读取yaml文件
    yaml_path = yaml_position_path
    print("读取yaml文件: {}".format(yaml_path))
    positions = read_yaml_file(yaml_path)
        
    if not positions:
        print("无法读取位置配置，程序退出")
        return

    # 初始位置自动修正
    for _ in range(2):
        correct_robot_position_by_pose(positions['origin'])
        time.sleep(1.5)  # 等待修正完成


    # 启动自动位置修正线程
    position_correction_thread = threading.Thread(target=run_position_correction)
    position_correction_thread.daemon = True  # 设置为守护线程，主线程结束时自动终止
    position_correction_thread.start()
    print("位置自动修正功能已启动")

    time.sleep(1.5)  # 等待线程启动完成
    

    
    # 获取当前位置
    current_pose = Get_current_pose()
    print("当前位置:")
    # print_pose(current_pose)
    
    # 打印位置字典的所有键，帮助调试
    print("位置配置包含以下键:")
    for key in positions.keys():
        print("键: {} (类型: {})".format(key, type(key)))
    
    # 尝试不同的方式获取货架一的位置
    for _ in range(2):
        shelf_position = None
        shelf_key = '货架1.{}'.format(_+1)
        
        shelf_position = positions[unicode(shelf_key, 'utf-8')] # type: ignore
        
        if not shelf_position:
            print("未找到货架一的位置信息，程序退出")
            return
        
        print("正在导航到货架1...")

        # set_xy_goal_tolerance(1)
        # set_yaw_goal_tolerance(1)
        # navigate_to_position(positions[unicode("安徽", 'utf-8')])
        # return
        # 设置导航容差
        set_xy_goal_tolerance()
        set_yaw_goal_tolerance()
        if navigate_to_position(shelf_position):
            print("已到达货架1")
            correct_robot_position()
            time.sleep(1.5)  # 等待校准完成
            
            # 调用take_box_main函数取盒子
            print("开始取盒子...")
            # province = take_box_main()
            print("取盒子完成")
            if not province:
                print("未检测到省份，程序退出")
                return
            
            # 导航到对应省份位置
            province_position = positions.get(province)
            if not province_position:
                print("未找到省份 {} 的位置信息，程序退出".format(province))
                return
            
            print("正在导航到 {}...".format(province))

            # 设置导航容差
            set_xy_goal_tolerance(0.05)
            set_yaw_goal_tolerance(0.1)
            if navigate_to_position(province_position):
                print("已到达 {}，完成配送任务".format(province))
                # 设置导航容差
                set_xy_goal_tolerance()
                set_yaw_goal_tolerance()
                correct_robot_position()
                put_box_main()
            else:
                print("导航到 {} 失败".format(province))
        else:
            print("导航到货架1失败")


if __name__ == '__main__':
    main()

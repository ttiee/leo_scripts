#!/usr/bin/env python
# -*- coding:utf-8 -*-

"""

功能：多点导航程序
该文件读取CSV格式的点位文件，依次导航到各个点位
- load_waypoints(): 从CSV文件加载点位数据
- navigate_to_waypoints(): 依次导航到所有点位
- wait_for_goal_reached(): 等待到达目标点
- main(): 主函数：执行多点导航

作者：周奕潇
创建日期：2025年3月30日

"""

import rospy
import csv
import time
import os
import sys

sys.path.append('/home/eaibot/scripts')

from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Pose
from Nav_Api import set_goal, cancel_goal, Get_current_pose, print_pose


def load_waypoints(csv_file):
    """
    功能：从CSV文件加载点位数据
    参数：
        csv_file: CSV文件路径
    返回：
        waypoints: 点位列表，每个点位包含7个参数(x,y,z,四元数x,y,z,w)
    
    CSV格式: timestamp,x,y,z,orientation_x,orientation_y,orientation_z,orientation_w
    """
    waypoints = []
    try:
        with open(csv_file, 'r') as f:
            reader = csv.reader(f)
            header = next(reader, None)  # 跳过标题行
            for row in reader:
                if len(row) >= 8:  # 确保每一行至少有8个值 (timestamp + 7个位姿参数)
                    try:
                        # 将字符串转换为浮点数，忽略timestamp列，只取位置和方向数据
                        # 正确的顺序是：timestamp,x,y,z,orientation_x,orientation_y,orientation_z,orientation_w
                        point = [float(row[1]), float(row[2]), float(row[3]), 
                                float(row[4]), float(row[5]), float(row[6]), float(row[7])]
                        waypoints.append(point)
                    except ValueError:
                        rospy.logwarn("忽略无效的行: {}".format(row))
                else:
                    rospy.logwarn("行格式不正确 (需要8个值): {}".format(row))
    except Exception as e:
        rospy.logerr("读取CSV文件出错: {}".format(e))
        return []
    
    rospy.loginfo("成功加载 {} 个点位".format(len(waypoints)))
    return waypoints


def wait_for_goal_reached(timeout=60):
    """
    功能：等待导航完成
    参数：
        timeout: 超时时间(秒)
    返回：
        bool: 成功到达返回True，超时返回False
    """
    start_time = time.time()
    rate = rospy.Rate(2)  # 2Hz检查频率
    
    goal_reached = False
    last_status = None
    
    while not rospy.is_shutdown() and (time.time() - start_time) < timeout:
        try:
            # 获取导航状态
            status_msg = rospy.wait_for_message("/move_base/status", GoalStatusArray, timeout=1)
            
            # 如果没有活动的目标，可能已经到达或未设置目标
            if not status_msg.status_list:
                # 获取当前位置，查看是否接近目标
                current_pose = Get_current_pose()
                # 这里可以添加位置验证的逻辑
                goal_reached = True
                break
            
            # 获取最新的目标状态
            status = status_msg.status_list[-1].status
            
            # 如果状态改变，打印状态信息
            if status != last_status:
                status_text = {
                    0: "等待中",
                    1: "正在执行",
                    2: "已完成",
                    3: "被抢占",
                    4: "被取消",
                    5: "被拒绝"
                }.get(status, "未知状态({})".format(status))
                rospy.loginfo("导航状态: {}".format(status_text))
                last_status = status
            
            # 检查是否到达目标
            if status == 3:  # 目标被抢占
                rospy.loginfo("目标被新的导航目标取代")
                return True
            elif status == 4:  # 目标被取消
                rospy.loginfo("导航目标被取消")
                return False
            elif status == 2:  # 目标完成
                rospy.loginfo("成功到达目标点")
                return True
            
            rate.sleep()
            
        except Exception as e:
            rospy.logwarn("等待导航完成时出错: {}".format(e))
            rate.sleep()
    
    if (time.time() - start_time) >= timeout:
        rospy.logwarn("导航超时 ({0}秒)".format(timeout))
        return False
    
    return goal_reached


def navigate_to_waypoints(waypoints, wait_time=2.0):
    """
    功能：依次导航到所有点位
    参数：
        waypoints: 点位列表，每个点位格式为 [x,y,z,qx,qy,qz,qw]
        wait_time: 到达点位后的等待时间（秒）
    """
    if not waypoints:
        rospy.logerr("没有点位数据，无法导航")
        return False
    
    # 确保ROS节点已初始化
    if not rospy.core.is_initialized():
        rospy.init_node('multi_point_nav', anonymous=True)
    
    total_points = len(waypoints)
    successful_points = 0
    
    for i, point in enumerate(waypoints):
        rospy.loginfo("导航到点位 {}/{}".format(i+1, total_points))
        
        try:
            # 解包点位数据
            x, y, z, qx, qy, qz, qw = point
            
            # 设置导航目标
            rospy.loginfo("设置导航目标: x={0}, y={1}, z={2}, qx={3}, qy={4}, qz={5}, qw={6}".format(
                x, y, z, qx, qy, qz, qw))
            set_goal(x, y, z, qx, qy, qz, qw)
            
            # 等待导航完成
            if wait_for_goal_reached():
                rospy.loginfo("成功到达点位 {}".format(i+1))
                successful_points += 1
                
                # 到达点位后等待一段时间
                if wait_time > 0:
                    rospy.loginfo("等待 {} 秒...".format(wait_time))
                    time.sleep(wait_time)
            else:
                rospy.logwarn("未能到达点位 {}".format(i+1))
        
        except Exception as e:
            rospy.logerr("导航到点位 {} 时出错: {}".format(i+1, e))
            # 继续尝试下一个点位
    
    # Python 2.7 的除法运算会得到整数结果，需要确保浮点数除法
    success_rate = (float(successful_points) / total_points) * 100 if total_points > 0 else 0
    rospy.loginfo("导航完成: 成功率 {:.1f}% ({}/{})".format(success_rate, successful_points, total_points))
    return successful_points == total_points


def main():
    """主函数：执行多点导航"""
    # 初始化ROS节点
    rospy.init_node('multi_point_nav', anonymous=True)
    csv_file = "robot_position/robot_positions.csv"
    
    # 加载点位
    waypoints = load_waypoints(csv_file)
    if not waypoints:
        rospy.logerr("未加载到有效的点位数据")
        return
    
    # 开始导航
    rospy.loginfo("开始导航，共 {} 个点位".format(len(waypoints)))
    navigate_to_waypoints(waypoints)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("程序被中断")
    except Exception as e:
        rospy.logerr("程序出错: {}".format(e))
        cancel_goal()  # 出错时取消导航目标

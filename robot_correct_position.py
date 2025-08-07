#!/usr/bin/env python
#-*- coding:utf-8 -*-

"""
功能：获取机器人当前位置并发布到/initialpose话题以修正机器人位置
      该文件包含了获取机器人位姿的函数
      - correct_robot_position(): 获取当前位置并发布到/initialpose话题
      - correct_robot_position_by_pose(): 将pose位置发布到/initialpose话题以修正机器人位置

作者：仝铁
创建日期：2025年3月30日
修改日期：2025年3月30日

"""


import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import time

# 导入Nav_Api模块
from Nav_Api import Get_current_pose, print_pose


def correct_robot_position():
    """
    获取当前位置并发布到/initialpose话题以修正机器人位置
    
    返回值:
        bool: 成功返回True，失败返回False
    """
    try:
        # 确保ROS节点已初始化
        if not rospy.core.is_initialized():
            rospy.init_node('position_corrector', anonymous=True)
            print("ROS节点初始化成功")
    except rospy.exceptions.ROSException as e:
        print("ROS节点已经初始化: {}".format(e))
    
    try:
        # 获取当前位置
        current_pose = Get_current_pose()
        if current_pose is None:
            print("无法获取当前位置")
            return False
            
        # 打印当前位置
        # print("当前位置:")
        # print_pose(current_pose)
        
        # 创建一个发布者用于发送初始位置
        pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
        # 等待发布者连接
        time.sleep(0.5)
        
        # 创建PoseWithCovarianceStamped消息
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.stamp = rospy.Time.now()
        initial_pose.header.frame_id = "map"  # 使用地图坐标系
        
        # 填充位姿信息
        initial_pose.pose.pose.position.x = current_pose.position.x
        initial_pose.pose.pose.position.y = current_pose.position.y
        initial_pose.pose.pose.position.z = current_pose.position.z
        initial_pose.pose.pose.orientation.x = current_pose.orientation.x
        initial_pose.pose.pose.orientation.y = current_pose.orientation.y
        initial_pose.pose.pose.orientation.z = current_pose.orientation.z
        initial_pose.pose.pose.orientation.w = current_pose.orientation.w
        
        # 设置协方差（表示位置估计的确定性）
        # 使用默认较小的协方差值
        for i in range(36):
            if i == 0 or i == 7 or i == 14:  # x, y, z位置的对角线元素
                initial_pose.pose.covariance[i] = 0.1
            elif i == 21 or i == 28 or i == 35:  # 旋转的对角线元素
                initial_pose.pose.covariance[i] = 0.05
            else:
                initial_pose.pose.covariance[i] = 0.0
        
        # 发布位姿
        pub.publish(initial_pose)
        # print("已发送位置修正信息到/initialpose话题")
        time.sleep(0.5)  # 确保消息被处理
        
        return True
        
    except Exception as e:
        print("位置修正失败: {}".format(e))
        return False
    

def correct_robot_position_by_pose(pose):
    """
    将pose位置发布到/initialpose话题以修正机器人位置
    
    返回值:
        bool: 成功返回True，失败返回False
    """
    try:
        # 确保ROS节点已初始化
        if not rospy.core.is_initialized():
            rospy.init_node('position_corrector', anonymous=True)
            print("ROS节点初始化成功")
    except rospy.exceptions.ROSException as e:
        print("ROS节点已经初始化: {}".format(e))
    
    try:
        # 创建一个发布者用于发送初始位置
        pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
        # 等待发布者连接
        time.sleep(0.5)
        
        # 创建PoseWithCovarianceStamped消息
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.stamp = rospy.Time.now()
        initial_pose.header.frame_id = "map"  # 使用地图坐标系
        
        # 填充位姿信息
        initial_pose.pose.pose.position.x = pose.get('x', 0.0)
        initial_pose.pose.pose.position.y = pose.get('y', 0.0)
        initial_pose.pose.pose.position.z = pose.get('z', 0.0)
        initial_pose.pose.pose.orientation.x = pose.get('orientation_x', 0.0)
        initial_pose.pose.pose.orientation.y = pose.get('orientation_y', 0.0)
        initial_pose.pose.pose.orientation.z = pose.get('orientation_z', 0.0)
        initial_pose.pose.pose.orientation.w = pose.get('orientation_w', 1.0)

        # 设置协方差（表示位置估计的确定性）
        # 使用默认较小的协方差值
        for i in range(36):
            if i == 0 or i == 7 or i == 14:  # x, y, z位置的对角线元素
                initial_pose.pose.covariance[i] = 0.1
            elif i == 21 or i == 28 or i == 35:  # 旋转的对角线元素
                initial_pose.pose.covariance[i] = 0.05
            else:
                initial_pose.pose.covariance[i] = 0.0
        
        # 发布位姿
        pub.publish(initial_pose)
        
        return True
        
    except Exception as e:
        print("位置修正失败: {}".format(e))
        return False


# 如果作为主程序运行，则执行位置修正
if __name__ == "__main__":
    correct_robot_position()


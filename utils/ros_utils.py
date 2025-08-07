#!/usr/bin/env python
# -*- coding:utf-8 -*-

"""
功能：ROS工具函数，处理ROS节点初始化相关问题

创建日期：2025年3月28日
修改日期：2025年3月29日

"""

import rospy

def ensure_ros_node_initialized(node_name='robot_node', anonymous=True):
    """
    确保ROS节点已经初始化，如果没有则进行初始化
    
    参数:
        node_name (str): 节点名称
        anonymous (bool): 是否使用匿名节点
        
    返回:
        bool: 如果节点已经初始化或者成功初始化则返回True
    """
    try:
        if not rospy.core.is_initialized():
            rospy.init_node(node_name, anonymous=anonymous)
            print("ROS节点'{}'初始化成功".format(node_name))
            return True
        else:
            print("ROS节点已经初始化")
            return True
    except rospy.exceptions.ROSException as e:
        print("ROS节点初始化错误: {}".format(e))
        # 如果错误是因为节点已经初始化，我们可以继续
        if "already been called" in str(e):
            return True
        return False

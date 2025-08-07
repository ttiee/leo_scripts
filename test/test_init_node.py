#!/usr/bin/env python3
#-*- coding:utf-8 -*-

"""
######### 说明：一个py程序只能初始化一个ROS节点！！！！！！！！ #########

作者：仝铁
创建日期：2025年3月28日
修改日期：2025年3月29日

"""

import rospy


if __name__ == "__main__":
    # 初始化 ROS 节点
    rospy.init_node('test_init', anonymous=True)
    
    # rospy.init_node('test_init2', anonymous=True)
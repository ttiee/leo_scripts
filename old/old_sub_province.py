#!/usr/bin/env python
#-*- coding:utf-8 -*-

"""
功能：包含一个调用一次就返回一次省份信息的函数

作者：仝铁
创建日期：2025年3月29日
修改日期：2025年3月29日

"""

import rospy
from std_msgs.msg import String
import time


def get_province_info():
    """
    订阅省份信息话题，获取省份信息并返回
    """
    try:
        # 创建ROS节点
        rospy.init_node('province_info_subscriber', anonymous=True)

        # 创建订阅者，订阅'/province_info'话题
        province = rospy.wait_for_message('/qrcode_info', String, timeout=5)
        # 打印接收到的省份信息
        print("接收到的省份信息: {}".format(province.data))
        return province.data
    except rospy.ROSException as e:
        print("ROS异常: {}".format(e))
        return None


if __name__ == '__main__':
    # 获取省份信息
    province_info = get_province_info()
    
    if province_info:
        print("获取到的省份信息: {}".format(province_info))
    else:
        print("未能获取到省份信息")

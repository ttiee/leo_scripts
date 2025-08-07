#!/usr/bin/env python
# -*- coding:utf-8 -*-

"""
功能：随机返回一个省份名称

作者：仝铁
创建日期：2025年3月28日
修改日期：2025年3月29日

"""

from __future__ import print_function

import random


def get_random_province():
    """
    随机返回一个省份名称
    """
    # provinces = [
    #     "北京", "天津", "上海", "重庆", "河北", "山西", "内蒙古", "辽宁",
    #     "吉林", "黑龙江", "江苏", "浙江", "安徽", "福建", "江西", "山东",
    #     "河南", "湖北", "湖南", "广东", "广西", "海南", "四川", "贵州",
    #     "云南", "西藏", "陕西", "甘肃", "青海", "宁夏", "新疆"
    # ]
    provinces = ["浙江", "安徽", "福建", "湖南"]
    
    return random.choice(provinces)

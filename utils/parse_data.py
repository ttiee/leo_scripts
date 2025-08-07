#!/usr/bin/env python
# -*- coding:utf-8 -*-

"""
功能：包含读取文件数据函数，位姿数据解析的函数

作者：仝铁
创建日期：2025年3月27日
修改日期：2025年3月30日

"""


import re
import csv
import ast  # 用于安全地转换字符串为列表

import io
import yaml  # 用于读取yaml文件


def parse_pose_data(pose_text):
    """
    解析位姿文本数据, 该函数用于解析Dobot的位姿文本数据
    
    参数:
    - pose_text: 原始位姿文本
    
    返回:
    - 解析后的位姿字典
    """
    pose_dict = {
        'result': pose_text.result,
        'x': pose_text.x,
        'y': pose_text.y,
        'z': pose_text.z,
        'r': pose_text.r,
        'jointAngle': pose_text.jointAngle
    }

    return pose_dict


def parse_pose_data_by_re(pose_text):
    """
    解析位姿文本数据（已废弃，使用parse_pose_data代替）
    
    参数:
    - pose_text: 原始位姿文本
    
    返回:
    - 解析后的位姿字典
    """
    pose_dict = {
        'result': None,
        'x': None,
        'y': None,
        'z': None,
        'r': None,
        'jointAngle': None
    }
    
    # 使用正则表达式解析
    pose_text = str(pose_text)
    lines = pose_text.split('\n')
    for line in lines:
        # 处理 result
        if line.startswith('result:'):
            pose_dict['result'] = line.split(':')[1].strip()
        
        # 处理坐标
        elif line.startswith('x:'):
            pose_dict['x'] = float(line.split(':')[1].strip())
        elif line.startswith('y:'):
            pose_dict['y'] = float(line.split(':')[1].strip())
        elif line.startswith('z:'):
            pose_dict['z'] = float(line.split(':')[1].strip())
        elif line.startswith('r:'):
            pose_dict['r'] = float(line.split(':')[1].strip())
        
        # 处理关节角度
        elif line.startswith('jointAngle:'):
            # 使用正则提取列表中的浮点数
            joint_angles = re.findall(r'[-+]?\d*\.\d+|\d+', line)
            pose_dict['jointAngle'] = [float(angle) for angle in joint_angles]
    
    return pose_dict


def read_pose_file(filename):
    """
    读取 CSV 文件中的位姿信息
    
    参数:
    - filename: CSV 文件名
    
    返回:
    - 位姿列表
    """
    poses = []
    try:
        with open(filename, 'rb') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                # 安全地将关节角度字符串转换为列表
                row['jointAngle'] = ast.literal_eval(row['jointAngle'])
                poses.append(row)
        return poses
    except Exception as e:
        print("读取文件时发生错误: {0}".format(e))
        return None


def read_yaml_file(file_path):
    """
    读取yaml文件
    """
    try:
        with io.open(file_path, 'r', encoding='utf-8') as file:
            data = yaml.safe_load(file)
            return data
    except Exception as e:
        print("读取yaml文件时出错: {}".format(e))
        return None

#!/usr/bin/env python
# -*- coding:utf-8 -*-

"""
功能：通过按键获取位置，命名点位并存储到YAML文件
      支持从模板YAML文件加载预定义的点位名称

说明：
################ 该文件已废弃，使用 yaml_position_recorder.py 替代 ##################

使用方法：
- 按空格键保存当前位置（会提示输入点位名称）
- 按 't' 从模板中选择点位进行更新
- 按 'l' 加载YAML模板文件
- 按 'f' 刷新当前位置
- 按 'p' 打印位置数据
- 按 'c' 清除位置数据
- 按 's' 退出并保存数据
- 按 'q' 退出（不保存数据）

作者：仝铁
创建日期：2025年3月28日
修改日期：2025年3月29日

"""

from __future__ import print_function

import os
import yaml
import time
import rospy
import sys

sys.path.append('/home/eaibot/scripts')
from utils import getch
from Nav_Api import Get_current_pose


class YamlPositionRecorder:
    def __init__(self, output_dir="/home/eaibot/scripts/robot_poses", 
                 template_file=None):
        self.output_dir = output_dir
        self.positions = {}  # 使用字典存储命名点位
        self.template_file = template_file
        
        # 确保输出目录存在
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
        
        # 初始化ROS节点
        if not rospy.core.is_initialized():
            rospy.init_node('yaml_position_recorder', anonymous=True)
        
        # 如果提供了模板文件，则加载它
        if template_file and os.path.exists(template_file):
            self.load_template(template_file)
    
    def load_template(self, template_file):
        """从YAML文件加载点位模板"""
        try:
            with open(template_file, 'r') as f:
                template_data = yaml.safe_load(f)
            
            if template_data and isinstance(template_data, dict):
                self.positions = template_data
                print("成功加载模板文件: {0}".format(template_file))
                print("已加载 {0} 个点位".format(len(self.positions)))
            else:
                print("模板文件格式错误或为空")
        except Exception as e:
            print("加载模板文件时出错: {0}".format(str(e)))
    
    def prompt_for_template_file(self):
        """提示用户输入模板文件路径"""
        print("\n请输入模板YAML文件的路径: ", end='')
        file_path = input().strip()
        
        if not file_path:
            print("未提供文件路径，操作取消")
            return False
        
        if not os.path.exists(file_path):
            print("文件不存在: {0}".format(file_path))
            return False
        
        self.load_template(file_path)
        self.template_file = file_path
        return True
    
    def select_position_from_template(self):
        """从已加载的模板中选择一个点位进行更新"""
        if not self.positions:
            print("没有已加载的点位模板，请先加载模板文件")
            return None
        
        # 显示可用点位
        print("\n可用的点位:")
        position_names = sorted(self.positions.keys())
        for i, name in enumerate(position_names):
            print("{0}. {1}".format(i+1, name))
        
        # 用户选择
        print("\n请输入点位编号(1-{0})或点位名称: ".format(len(position_names)), end='')
        selection = input().strip()
        
        # 处理选择
        selected_name = None
        try:
            # 尝试解析为索引
            idx = int(selection) - 1
            if 0 <= idx < len(position_names):
                selected_name = position_names[idx]
            else:
                print("无效的索引: {0}".format(selection))
                return None
        except ValueError:
            # 直接用作点位名称
            if selection in self.positions:
                selected_name = selection
            else:
                print("找不到点位: {0}".format(selection))
                return None
        
        print("已选择点位: {0}".format(selected_name))
        return selected_name
    
    def update_position_from_template(self):
        """更新模板中的点位"""
        selected_name = self.select_position_from_template()
        if not selected_name:
            return False
        
        try:
            # 获取机器人当前位置
            pose = Get_current_pose()
            
            # 更新点位信息，保留原始的时间戳
            original_timestamp = self.positions[selected_name].get('timestamp', time.time())
            
            position = {
                'timestamp': original_timestamp,
                'x': pose.position.x,
                'y': pose.position.y,
                'z': pose.position.z,
                'orientation_x': pose.orientation.x,
                'orientation_y': pose.orientation.y,
                'orientation_z': pose.orientation.z,
                'orientation_w': pose.orientation.w
            }
            
            # 更新点位
            self.positions[selected_name] = position
            
            print("点位已更新: 名称=\"{0}\", 坐标=({1:.3f}, {2:.3f}, {3:.3f})".format(
                selected_name, pose.position.x, pose.position.y, pose.position.z))
            return True
        except Exception as e:
            print("更新点位时出错: {0}".format(str(e)))
            return False
    
    def get_position_name(self):
        """获取用户输入的点位名称"""
        print("\n请输入点位名称: ", end='')
        name = input().strip()  # 使用raw_input兼容Python 2
        
        # 确保点位名称不为空
        while not name:
            print("点位名称不能为空，请重新输入: ", end='')
            name = input().strip()
            
        # 检查名称是否已存在
        if name in self.positions:
            print("警告：点位名称已存在，将覆盖原有点位")
            


if __name__ == "__main__":
    recorder = YamlPositionRecorder()
    recorder.run()

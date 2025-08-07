#!/usr/bin/env python
# -*- coding:utf-8 -*-

"""
功能：通过按键获取位置，命名点位并存储到YAML文件
      支持从模板YAML文件加载预定义的点位名称

使用方法：
- 按 't' 从模板中选择点位进行更新
- 按 'f' 刷新当前位置
- 按 'p' 打印位置数据
- 按 's' 退出并保存数据
- 按 'q' 退出（不保存数据）

作者：仝铁
创建日期：2025年3月28日
修改日期：2025年3月38日

"""

from __future__ import print_function

import os
import yaml
import time
import rospy
import io
import sys
import threading
from utils import getch
from Nav_Api import Get_current_pose
from robot_correct_position import correct_robot_position

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
        time.sleep(10)  # 每5秒检查一次位置
        # 调用位置修正函数
        correct_robot_position()


class YamlPositionRecorder:
    def __init__(self, output_dir="/home/eaibot/scripts/robot_position", 
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
            with io.open(template_file, 'r', encoding='utf-8') as f:
                template_data = yaml.safe_load(f)
            # print(template_data)
            
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
        global yaml_position_path
        
        if not os.path.exists(yaml_position_path):
            print("文件不存在: {0}".format(yaml_position_path))
            return False
        
        self.load_template(yaml_position_path)
        self.template_file = yaml_position_path
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
        selection = str(input()).strip()
        
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
    
    def save_to_yaml(self):
        """将所有位置保存到YAML文件"""
        if not self.positions:
            print("没有位置数据可保存")
            return
        
        # 如果使用了模板，优先保存回原模板文件
        if self.template_file and os.path.exists(os.path.dirname(self.template_file)):
            filename = self.template_file
            print("更新模板文件: {0}".format(filename))
        else:
            # 否则创建新文件
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(self.output_dir, "robot_positions_{0}.yaml".format(timestamp))
        
        with io.open(filename, 'w', encoding='utf-8') as yamlfile:
            yaml.dump(self.positions, yamlfile, default_flow_style=False, allow_unicode=True)
        
        print("位置数据已保存到: {0}".format(filename))
    
    def print_positions(self):
        """打印所有保存的点位"""
        if not self.positions:
            print("没有保存的点位数据")
            return
        
        print("\n已保存的点位数据:")
        print("-" * 50)
        for name, position in sorted(self.positions.items()):
            print(u"点位名称: {0}".format(name))
            print(u"  坐标: ({0:.3f}, {1:.3f}, {2:.3f})".format(
                position['x'], position['y'], position['z']))
            print(u"  方向四元数: ({0:.3f}, {1:.3f}, {2:.3f}, {3:.3f})".format(
                position['orientation_x'], position['orientation_y'], 
                position['orientation_z'], position['orientation_w']))
            print("-" * 50)
    

    def print_info(self):
        print("######## YAML点位记录器 ########")
        print("\t按键控制:")
        print("t: 从模板中选择点位进行更新")
        print("f: 刷新当前位置")
        print("p: 打印位置数据")
        print("s: 退出并保存数据")
        print("q: 退出(不保存数据)")


    def run(self):
        """运行程序"""
        
        if self.template_file:
            print("\n当前使用的模板文件: {0}".format(self.template_file))
            
        print("加载YAML模板文件")
        self.prompt_for_template_file()

        # 启动自动位置修正线程
        # position_correction_thread = threading.Thread(target=run_position_correction)
        # position_correction_thread.daemon = True  # 设置为守护线程，主线程结束时自动终止
        # position_correction_thread.start()
        # print("位置自动修正功能已启动")
        
        while True:
            self.print_info()
            try:
                # 显示当前位置
                pose = Get_current_pose()
                print("\n当前位置: ({0:.3f}, {1:.3f}, {2:.3f})".format(
                    pose.position.x, pose.position.y, pose.position.z))
                
                # 等待按键
                key = getch()
                
                if key == 'q':
                    print("退出程序，不保存数据")
                    break
                elif key == 's':
                    print("保存位置数据并退出")
                    self.save_to_yaml()
                    break
                elif key == 'p':
                    self.print_positions()
                elif key == 'f':
                    print("刷新当前位置")
                    continue
                elif key == 't':
                    print("从模板中选择点位进行更新")
                    self.update_position_from_template()  
                else:
                    print("未知按键: {0}".format(key))
            except KeyboardInterrupt:
                print("\n检测到键盘中断，退出程序")
                break
            except Exception as e:
                print("发生错误: {0}".format(str(e)))
                time.sleep(1)
        
        print("程序已退出")


if __name__ == "__main__":
    import sys
    
    # 检查是否提供了模板文件作为命令行参数
    template_file = None
    if len(sys.argv) > 1:
        template_file = sys.argv[1]
        if not os.path.exists(template_file):
            print("警告：指定的模板文件不存在: {0}".format(template_file))
            template_file = None
    
    recorder = YamlPositionRecorder(template_file=template_file)
    recorder.run()

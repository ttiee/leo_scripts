#!/usr/bin/env python
# -*- coding:utf-8 -*-

"""
功能：通过按键获取位置并存储到CSV文件

使用方法：
- 按空格键保存当前位置
- 按 'f' 刷新当前位置
- 按 'p' 打印位置数据
- 按 'c' 清除位置数据
- 按 's' 退出并保存数据
- 按 'q' 退出（不保存数据）

作者：仝铁
创建日期：2025年3月27日
修改日期：2025年3月28日

"""
from __future__ import print_function

import os
import csv
import time
import rospy
import sys

sys.path.append('/home/eaibot/scripts')

from utils import getch
from Nav_Api import Get_current_pose


class PositionRecorder:
    def __init__(self, output_dir="/home/eaibot/scripts/robot_position"):
        self.output_dir = output_dir
        self.positions = []
        
        # 确保输出目录存在
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
        
        # 初始化ROS节点
        if not rospy.core.is_initialized():
            rospy.init_node('position_recorder', anonymous=True)
    
    def save_position(self):
        """保存当前位置"""
        try:
            # 获取机器人当前位置
            pose = Get_current_pose()
            timestamp = time.time()
            position = {
                'timestamp': timestamp,
                'x': pose.position.x,
                'y': pose.position.y,
                'z': pose.position.z,
                'orientation_x': pose.orientation.x,
                'orientation_y': pose.orientation.y,
                'orientation_z': pose.orientation.z,
                'orientation_w': pose.orientation.w
            }
            self.positions.append(position)
            print("位置已保存: ({0}, {1}, {2})".format(pose.position.x, pose.position.y, pose.position.z))
            return True
        except Exception as e:
            print("保存位置时出错: {0}".format(str(e)))
            return False
    
    def save_to_csv(self):
        """将所有位置保存到CSV文件"""
        if not self.positions:
            print("没有位置数据可保存")
            return
        
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(self.output_dir, "robot_positions_{0}.csv".format(timestamp))
        
        with open(filename, 'wb') as csvfile:
            fieldnames = ['timestamp', 'x', 'y', 'z', 'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            
            writer.writeheader()
            for position in self.positions:
                writer.writerow(position)
        
        print("位置数据已保存到: {0}".format(filename))
    
    def run(self):
        """运行程序"""
        print("按键控制:")
        print("空格键: 保存当前位置")
        print("f: 刷新当前位置")
        print("p: 打印位置数据")
        print("c: 清除位置数据")
        print("s: 退出并保存数据")
        print("q: 退出(不保存数据)")
        print("按任意键继续...")
        
        while True:
            try:
                # 显示当前位置
                pose = Get_current_pose()
                print("当前位置: ({0}, {1}, {2})".format(pose.position.x, pose.position.y, pose.position.z))
                
                # 等待按键
                key = getch()
                
                if key == 'q':
                    break
                elif key == 's':
                    print("保存位置数据")
                    self.save_to_csv()
                    break
                elif key == 'c':
                    print("清除位置数据")
                    self.positions = []
                elif key == 'p':
                    print("打印位置数据")
                    if not self.positions:
                        print("没有位置数据")
                    else:
                        print("位置数据:")
                        for position in self.positions:
                            print(position)
                elif key == 'f':
                    print("刷新当前位置")
                    continue
                elif key == ' ':
                    self.save_position()
            except KeyboardInterrupt:
                break
            except Exception as e:
                print("发生错误: {0}".format(str(e)))
                time.sleep(1)
        
        # self.save_to_csv()
        print("程序已退出")


if __name__ == "__main__":
    recorder = PositionRecorder()
    recorder.run()

#!/usr/bin/env python
# -*- coding:utf-8 -*-

"""
播放指定文件夹的mp3文件语音

作者：human-made Fe
创建日期：2025年5月17日
修改日期：2025年5月17日

"""

import os
import subprocess

def play_province_audio(province_name, mp3_folder_path):
    """
    根据省份名查找并播放对应的MP3文件
    
    参数:
        province_name: 省份名称，如"北京"、"上海"等
        mp3_folder_path: MP3文件所在的文件夹路径
        
    返回:
        播放成功返回True，否则返回False
    """
    if not os.path.exists(mp3_folder_path) or not os.path.isdir(mp3_folder_path):
        print("错误：指定的文件夹路径不存在或不是一个文件夹！")
        return False
    
    # 在文件夹中查找匹配省份名的MP3文件
    matching_files = []
    for filename in os.listdir(mp3_folder_path):
        if filename.lower().endswith('.mp3') and province_name in filename:
            matching_files.append(filename)
    
    if not matching_files:
        print("未找到包含省份 '{}' 的MP3文件".format(province_name))
        return False
    
    # 如果找到多个匹配文件，使用第一个
    if len(matching_files) > 1:
        print("找到多个匹配的文件，将播放第一个: {}".format(matching_files[0]))
    
    mp3_path = os.path.join(mp3_folder_path, matching_files[0])
    
    try:
        # 使用mpg123播放MP3文件，适用于Ubuntu
        cmd = ['mpg123', mp3_path]
        subprocess.call(cmd)
        return True
    except Exception as e:
        print("播放音频时出错: {}".format(e))
        print("请确保已安装mpg123。可以通过运行 'sudo apt-get install mpg123' 来安装")
        return False

# 示例用法
if __name__ == "__main__":
    # 测试函数
    folder_path = "/home/eaibot/scripts/audio"  # 替换为实际的MP3文件夹路径
    province = "江苏"  # 替换为要播放的省份名称
    play_province_audio(province, folder_path)

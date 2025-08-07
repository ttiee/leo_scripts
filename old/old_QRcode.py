#!/usr/bin/env python
#-*- coding:utf-8 -*-

"""
功能：通过opencv读取摄像头图像，检测二维码并打印二维码数据和类型

作者：仝铁
创建日期：2025年3月29日

"""

from __future__ import print_function  # 兼容Python 2.7的print函数
import cv2
import numpy as np
from pyzbar.pyzbar import decode
import sys

# 设置默认编码为UTF-8
reload(sys) # type: ignore
sys.setdefaultencoding('utf-8')

def main():
    # 初始化摄像头
    cap = cv2.VideoCapture(0)  # 0表示默认摄像头，如果有多个摄像头可以尝试1,2等
    
    if not cap.isOpened():
        print("无法打开摄像头")
        return
    
    print("按 'q' 键退出程序")
    
    # 移除不必要的字体加载代码，因为不再显示图像
    
    while True:
        # 读取一帧图像
        ret, frame = cap.read()
        
        if not ret:
            print("无法获取图像帧")
            break
        
        # 二维码检测
        detected_qrs = decode(frame)
        print("检测到二维码数量: {}".format(len(detected_qrs)))
        
        # 处理每个检测到的二维码
        if detected_qrs:
            for qr in detected_qrs:
                # 获取二维码数据并正确处理编码
                try:
                    # 尝试UTF-8解码
                    qr_data = qr.data.decode('utf-8')
                except UnicodeDecodeError:
                    try:
                        # 如果UTF-8失败，尝试GBK解码（常用于中文）
                        qr_data = qr.data.decode('gbk')
                    except UnicodeDecodeError:
                        # 如果都失败，使用latin-1（不会失败，但可能显示不正确）
                        qr_data = qr.data.decode('latin-1')
                
                qr_type = qr.type
                
                # 修改为确保字符串编码正确处理
                print(u"检测到二维码: {0} (类型: {1})".format(qr_data, qr_type).encode('utf-8'))
    
    # 释放资源
    cap.release()

if __name__ == "__main__":
    main()
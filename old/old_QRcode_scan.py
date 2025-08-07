#!/usr/bin/env python
#-*- coding:utf-8 -*-

"""
功能：使用OpenCV和pyzbar库扫描二维码

"""

from __future__ import print_function  # 兼容Python 2.7的print函数

from pyzbar.pyzbar import decode
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class QRCodeScanner:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('qrcode_scanner', anonymous=True)
        
        # 创建CV Bridge
        self.bridge = CvBridge()
        
        # 创建图像订阅者
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        
        print("二维码扫描器已启动")
        print("按Ctrl+C退出程序")
    
    def image_callback(self, data):
        try:
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return
        
        # 二维码检测
        detected_qrs = decode(cv_image)
        
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


def main():
    # 创建QR码扫描器实例
    scanner = QRCodeScanner()
    
    # 让ROS节点保持运行
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("程序关闭")


if __name__ == "__main__":
    main()
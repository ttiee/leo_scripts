#!/usr/bin/env python
#-*- coding:utf-8 -*-

"""
功能：使用OpenCV和pyzbar库扫描二维码，使用话题发布省份信息

作者：仝铁
创建日期：2025年3月29日
修改日期：2025年3月29日

"""

from __future__ import print_function  # 兼容Python 2.7的print函数
from pyzbar.pyzbar import decode
import sys
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import json

# 设置默认编码为UTF-8
reload(sys) # type: ignore
sys.setdefaultencoding('utf-8')


class QRCodeScanner:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('qrcode_scanner', anonymous=True)
        
        # 创建CV Bridge
        self.bridge = CvBridge()
        
        # 创建统一的二维码信息发布者
        self.qrcode_info_pub = rospy.Publisher('/qrcode_info', String, queue_size=10)
        
        # 创建图像订阅者
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        
        # 省份列表
        self.provinces = [
            "北京", "上海", "天津", "重庆", "河北", "山西", "辽宁", "吉林", "黑龙江", 
            "江苏", "浙江", "安徽", "福建", "江西", "山东", "河南", "湖北", "湖南", 
            "广东", "海南", "四川", "贵州", "云南", "陕西", "甘肃", "青海", "台湾", 
            "内蒙古", "广西", "西藏", "宁夏", "新疆", "香港", "澳门"
        ]
        
        print("二维码扫描器已启动")
        print("按Ctrl+C退出程序")
    
    def extract_province(self, text):
        """从文本中提取省份信息"""
        for province in self.provinces:
            if province in text:
                return province
        return None
    
    def image_callback(self, data):
        try:
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return
        
        # 二维码检测
        detected_qrs = decode(cv_image)
        
        # 获取二维码数量
        qr_count = len(detected_qrs)
        
        if qr_count > 0:
            print("检测到{0}个二维码".format(qr_count))
        
        # 用于记录检测到的所有省份
        detected_provinces = []
        
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
                
                # 尝试提取省份信息
                province = self.extract_province(qr_data)
                if province:
                    print(u"检测到省份: {0}".format(province).encode('utf-8'))
                    # 将省份添加到列表中
                    if province not in detected_provinces:
                        detected_provinces.append(province)
            
            # 如果检测到省份，发布信息
            if detected_provinces:
                # 创建包含所有信息的字典
                qrcode_info = {
                    "count": qr_count,
                    "provinces": detected_provinces
                }
                
                # 转换为JSON字符串
                info_json = json.dumps(qrcode_info, ensure_ascii=False)
                print(u"二维码信息: {0}".format(info_json).encode('utf-8'))
                
                # 发布到统一话题
                self.qrcode_info_pub.publish(info_json)


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
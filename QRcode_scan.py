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
import json

# 设置默认编码为UTF-8
if sys.version_info[0] < 3:
    reload(sys)
    sys.setdefaultencoding('utf-8')


def get_province_info():
    """
    通过订阅ROS话题/usb_cam/image_raw直接获取省份信息并返回
    """
    try:
        import threading
        import time
        
        # 使用全局变量存储图像和状态
        global_data = {
            'image': None,
            'received': False,
            'provinces': [],
            'count': 0,
            'error': None
        }
        
        # 定义图像回调函数
        def image_callback(data):
            try:
                # 创建CV Bridge
                bridge = CvBridge()
                # 将ROS图像消息转换为OpenCV格式
                cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
                
                # 获取省份列表
                provinces = [
                    "北京", "上海", "天津", "重庆", "河北", "山西", "辽宁", "吉林", "黑龙江", 
                    "江苏", "浙江", "安徽", "福建", "江西", "山东", "河南", "湖北", "湖南", 
                    "广东", "海南", "四川", "贵州", "云南", "陕西", "甘肃", "青海", "台湾", 
                    "内蒙古", "广西", "西藏", "宁夏", "新疆", "香港", "澳门"
                ]
                
                # 提取省份函数
                def extract_province(text):
                    for province in provinces:
                        if province in text:
                            return province
                    return None
                
                # 二维码检测
                detected_qrs = decode(cv_image)
                
                # 获取二维码数量
                qr_count = len(detected_qrs)
                
                # 用于记录检测到的所有省份
                detected_provinces = []
                
                # 处理每个检测到的二维码
                if detected_qrs:
                    for qr in detected_qrs:
                        # 获取二维码数据并正确处理编码
                        try:
                            qr_data = qr.data.decode('utf-8')
                        except UnicodeDecodeError:
                            try:
                                qr_data = qr.data.decode('gbk')
                            except UnicodeDecodeError:
                                qr_data = qr.data.decode('latin-1')
                        
                        # 尝试提取省份信息
                        province = extract_province(qr_data)
                        if province and province not in detected_provinces:
                            detected_provinces.append(province)
                
                # 设置全局数据
                global_data['provinces'] = detected_provinces
                global_data['count'] = qr_count
                global_data['received'] = True
                
            except CvBridgeError as e:
                global_data['error'] = str(e)
                global_data['received'] = True
            except Exception as e:
                global_data['error'] = str(e)
                global_data['received'] = True
        
        # 初始化临时ROS节点
        if not rospy.core.is_initialized():
            rospy.init_node('qrcode_info_getter', anonymous=True)
        
        # 创建临时订阅者
        image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)
        
        # 等待图像接收，最多等待3秒
        timeout = time.time() + 3.0
        while not global_data['received'] and time.time() < timeout:
            time.sleep(0.1)
        
        # 取消订阅
        image_sub.unregister()
        
        # 检查是否成功接收图像
        if not global_data['received']:
            return json.dumps({"error": "获取图像超时"}, ensure_ascii=False)
        
        # 检查是否有错误
        if global_data['error']:
            return json.dumps({"error": global_data['error']}, ensure_ascii=False)
        
        # 返回结果
        result = {
            "count": global_data['count'],
            "provinces": global_data['provinces']
        }
        
        return json.dumps(result, ensure_ascii=False)
        
    except Exception as e:
        error_msg = {"error": str(e)}
        return json.dumps(error_msg, ensure_ascii=False)


def parse_province_info(json_str):
    """
    解析get_province_info()函数返回的JSON字符串，提取省份信息
    
    参数:
        json_str: get_province_info()返回的JSON字符串
        
    返回:
        如果成功，返回一个包含省份列表的列表
        如果失败，返回None和错误信息
    """
    try:
        # 解析JSON字符串
        data = json.loads(json_str)
        
        # 检查是否有错误字段
        if "error" in data:
            return None, "错误: " + data["error"]
        
        # 提取省份列表
        if "provinces" in data:
            provinces = data["provinces"]
            count = data.get("count", 0)
            
            if count > 0 and len(provinces) > 0:
                return provinces, None
            else:
                return [], "未检测到省份信息"
        else:
            return None, "返回数据格式错误，缺少provinces字段"
            
    except ValueError as e:
        return None, "JSON解析错误: " + str(e)
    except Exception as e:
        return None, "解析省份信息时发生错误: " + str(e)


def test_func():
    """
    测试函数：测试api函数
    """
    # 获取省份信息
    province_info = get_province_info()
    
    if province_info:
        print("获取到的原始信息: {}".format(province_info))
        
        # 解析省份信息
        provinces, error = parse_province_info(province_info)
        if provinces is not None:
            print("解析到的省份: {}".format(", ".join(provinces)))
        else:
            print(error)
    else:
        print("未能获取到省份信息")


if __name__ == "__main__":
    # main()
    test_func()
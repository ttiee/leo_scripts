#!/usr/bin/env python
# -*- coding:utf-8 -*-

"""
功能：读取csv位姿文件，依次到达一系列点位，并在特殊点位处取盒子，然后放置到指定位置

作者：仝铁
创建日期：2025年3月27日
修改日期：2025年3月30日

"""

import os
import time
import sys
import threading
from collections import Counter
# import Queue as queue  # Python 2.7中使用Queue模块
if sys.version_info[0] < 3:
    import Queue as queue
else:
    import queue

sys.path.append('/home/eaibot/scripts')

from Dobot.Dobot_Api import f_SetPTPCmd, f_SetEndEffectorSuctionCup
from utils import read_pose_file    # 读取csv位姿文件
from QRcode_scan import get_province_info, parse_province_info
from robot_move_distance import SimpleForward

# csv文件的位置
csv_file_name = '/home/eaibot/scripts/dobot_poses/poses_take-v1.4.csv'


def detect_province_in_thread(stop_event, province_results):
    """
    在后台线程中持续检测省份信息
    Args:
        stop_event: 停止线程的事件
        province_results: 存储检测结果的列表
    """
    print("启动省份检测线程")
    
    while not stop_event.is_set():
        try:
            # 使用主线程安全的方式获取信息
            try:
                json_info = get_province_info()  # 获取省份信息
                provinces, error = parse_province_info(json_info)
                
                if provinces and len(provinces) > 0:
                    province = provinces[0]
                    print("线程检测到省份: {0}".format(province))
                    province_results.append(province)
                elif error:
                    print("二维码扫描错误: {0}".format(error))
            except Exception as scan_error:
                print("二维码扫描错误: {0}".format(scan_error))
                # 如果是信号错误，等待更长时间避免频繁错误
                if "signal only works in main thread" in str(scan_error):
                    print("检测到信号错误，可能是在非主线程中使用了不支持的功能")
                    time.sleep(1.0)  # 稍微等久一点
                
            time.sleep(0.5)  # 短暂休眠，避免过度占用CPU
        except Exception as e:
            print("省份检测线程出错: {0}".format(e))
    
    print("省份检测线程已停止")


def move_to_poses(poses, detect_province=False):
    """
    依次移动到指定的点位，并在特定点位处取盒子"
    """
    print("开始移动到预设点位")
    province_detected = None
    
    # 如果需要检测省份，启动后台线程
    province_results = []
    stop_detect_thread = threading.Event()
    detect_thread = None
    
    # 在机械臂移动前先尝试直接在主线程中获取省份信息
    if detect_province:
        print("在主线程中尝试获取省份信息")
        try:
            json_info = get_province_info()
            provinces, error = parse_province_info(json_info)
            if provinces and len(provinces) > 0:
                for province in provinces:
                    province_results.append(province)
                    print("主线程检测到省份: {0}".format(province))
        except Exception as e:
            print("主线程获取省份信息失败: {0}".format(e))
        
        # 启动后台线程继续尝试
        detect_thread = threading.Thread(
            target=detect_province_in_thread,
            args=(stop_detect_thread, province_results)
        )
        detect_thread.daemon = True
        detect_thread.start()
    
    for i, pose in enumerate(poses, 1):
        # 打印当前点位信息
        print("\n移动到第 {0} 个点位:".format(i))
        print("坐标 - X: {0}, Y: {1}, Z: {2}, R: {3}".format(
            pose['x'], pose['y'], pose['z'], pose['r']
        ))

        # 调用 PTP 移动命令
        try:
            result = f_SetPTPCmd(
                float(pose['x']),
                float(pose['y']),
                float(pose['z']),
                float(pose['r'])
            )
            print("移动结果: {0}".format(result))
        except Exception as e:
            print("移动时发生错误: {0}".format(e))

        time.sleep(3)  # 等待 3 秒

        # 如果是第三个点位，则取盒子
        if i == 3:
            # 后退 0.08 米
            robot = SimpleForward()
            robot.move_distance(-0.08)
            robot.move_distance(-0.08)
            time.sleep(0.5)  # 等待1秒

        elif i == 4:
            print("在第4个点位取盒子")
            f_SetEndEffectorSuctionCup(True)
            time.sleep(0.5)  # 等待1秒

        elif i == 5:
            # 前进 0.08 米
            robot = SimpleForward()
            robot.move_distance(0.08)
            robot.move_distance(0.08)
            time.sleep(0.5)  # 等待1秒

        # 如果是第六个点位，则放置盒子
        if i == 6:
            print("在第7个点位放置盒子")
            f_SetEndEffectorSuctionCup(False)
            time.sleep(1)  # 等待1秒
        
        # 在关键位置时在主线程再次尝试获取省份
        if detect_province and (i == 1 or i == 2 or i == 3):
            print("在第{}个点位主线程再次尝试获取省份信息".format(i))
            try:
                json_info = get_province_info()
                provinces, error = parse_province_info(json_info)
                if provinces and len(provinces) > 0:
                    for province in provinces:
                        province_results.append(province)
                        print("主线程在点位{}检测到省份: {}".format(i, province))
            except Exception as e:
                print("主线程获取省份信息失败: {}".format(e))
        
        # 如果是最后一个点位，则结束
        if i == len(poses):
            print("已到达最后一个点位")
            break
    
    # 停止检测线程
    if detect_province and detect_thread:
        print("停止省份检测线程...")
        stop_detect_thread.set()
        try:
            # Python 2.7中join可能不支持timeout参数
            detect_thread.join(timeout=2)  # 最多等待2秒
        except TypeError:
            # 如果join不支持timeout，使用另一种方法确保线程结束
            start_time = time.time()
            while detect_thread.isAlive() and time.time() - start_time < 2:
                time.sleep(0.1)
        
        # 统计检测结果，选择出现次数最多的省份
        if province_results:
            counter = Counter(province_results)
            # Python 2.7的Counter.most_common返回格式可能不同，确保兼容
            most_common_list = counter.most_common(1)
            if most_common_list and len(most_common_list) > 0:
                most_common_item = most_common_list[0]
                if len(most_common_item) > 1:
                    province_detected = most_common_item[0]
                    print("最终检测结果: {0} (出现 {1} 次)".format(province_detected, most_common_item[1]))
                    print("所有检测结果统计: {0}".format(dict(counter)))
                else:
                    print("无法获取有效的最常见省份")
            else:
                print("无法获取检测结果统计")
        else:
            print("未检测到任何省份信息")
    
    return province_detected


def main():
    # 选择位姿文件
    filename = csv_file_name

    if filename is None:
        print("无该文件：{0}".format(filename))
        return

    # 读取位姿
    poses = read_pose_file(filename)

    if poses:
        # 开始移动到点位
        province = move_to_poses(poses, detect_province=True)
        print(province)

    return province


if __name__ == '__main__':
    main()
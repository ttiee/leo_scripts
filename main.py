#!/usr/bin/env python3.8
#-*- coding:utf-8 -*-

"""
main.py

1. 启动 roslaunch dobot DobotServer.launch 
2. 启动 roslaunch usb_cam usb_cam-test.launch
"""

import os
import subprocess
import time


def start_roslaunch(package, launch_file):
    """
    启动 roslaunch 命令
    :param package: ROS 包名
    :param launch_file: 启动文件名
    """
    command = f"roslaunch {package} {launch_file}"
    process = subprocess.Popen(command, shell=True)
    return process


def main():
    # 启动 DobotServer.launch
    dobot_process = start_roslaunch('dobot', 'DobotServer.launch')
    time.sleep(5)  # 等待 DobotServer 启动

    # 启动 usb_cam-test.launch
    usb_cam_process = start_roslaunch('usb_cam', 'usb_cam-test.launch')
    
    try:
        dobot_process.wait()
        usb_cam_process.wait()
    except KeyboardInterrupt:
        print("Stopping processes...")
        dobot_process.terminate()
        usb_cam_process.terminate()
        dobot_process.wait()
        usb_cam_process.wait()


if __name__ == "__main__":
    main()
    print("Starting ROS nodes...")
    print("DobotServer and usb_cam nodes are running.")
    print("Press Ctrl+C to stop the processes.")
    while True:
        time.sleep(1)
        # Keep the script running to maintain the ROS nodes
        # This allows the user to stop the processes with Ctrl+C
        # You can add additional logic here if needed
        # For example, monitoring the status of the nodes or logging information
        pass
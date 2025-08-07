#!/usr/bin/env python
#-*- coding:utf-8 -*-

"""
Dobot机械臂的API接口
该文件包含了Dobot机械臂的API接口函数

- f_SetHomeCmd(): 机械臂回到零点
- f_SetHomeParams(): 设置机械臂零点
- goto_home_and_set_home(): 机械臂回到零点并设置零点
- f_SetQueuedCmdStartExec(): 开始执行队列命令
- f_GetPose(): 获取当前机械臂的位置
- f_SetPTPCmd(x, y, z, r, isQueued=1): 设置机械臂的位置
- f_SetEndEffectorSuctionCup(suck, enableCtrl=1, isQueued=1): 设置机械臂的吸盘
- f_ClearAllAlarmsState(): 清除所有报警状态
- test_func(): 测试函数
- main(): 主函数

作者：仝铁
创建日期：2025年3月27日
修改日期：2025年3月28日

"""

import rospy
import time
from dobot.srv import SetHOMECmd, SetHOMEParams, SetQueuedCmdStartExec, GetPose, SetPTPCmd, SetEndEffectorSuctionCup, ClearAllAlarmsState


def f_SetHomeCmd():
    """
    功能：机械臂回到零点
    return: result & queuedCmdIndex
    """
    rospy.init_node('myDobot')
    rospy.wait_for_service('DobotServer/SetHOMECmd')   #指向服务，进行等待连接
    print("开始返回零点")
    try:
        client = rospy.ServiceProxy('DobotServer/SetHOMECmd',SetHOMECmd)  #创建服务客户端
        response = client()  #发送服务请求
        # print(response)
        return response
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def f_SetHomeParams():
    """
    功能：设置机械臂零点
    return: result & queuedCmdIndex
    """
    rospy.init_node('myDobot')
    rospy.wait_for_service('DobotServer/SetHOMEParams')   #指向服务，进行等待连接
    print("Set Home Params")
    try:
        client = rospy.ServiceProxy('DobotServer/SetHOMEParams',SetHOMEParams)  #创建服务客户端
        response = client(150, 0, 0, 0, False)  #发送服务请求
        print(response)
        return response
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def goto_home_and_set_home(sleep_time=10):
    """
    功能：机械臂回到零点并设置零点
    return: None
    """
    print(f_SetHomeCmd())
    for i in range(10):
        print(f_GetPose())
        time.sleep(2)
    time.sleep(sleep_time)
    
    print(f_SetHomeParams())
    print(f_GetPose())


def f_SetQueuedCmdStartExec():
    """
    功能：开始执行队列命令
    return: result & queuedCmdIndex
    """
    rospy.init_node('myDobot')
    rospy.wait_for_service('DobotServer/SetQueuedCmdStartExec')   #指向服务，进行等待连接
    try:
        client = rospy.ServiceProxy('DobotServer/SetQueuedCmdStartExec',SetQueuedCmdStartExec)  #创建服务客户端
        response = client()  #发送服务请求
        # print(response)
        return response
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def f_GetPose():
    """
    功能：获取当前机械臂的位置
    return: 返回当前机械臂的位置
    ['result','x','y','z','r','jointAngle']
    """

    rospy.init_node('myDobot')
    rospy.wait_for_service('DobotServer/GetPose')   #指向服务，进行等待连接
    try:
        client = rospy.ServiceProxy('DobotServer/GetPose',GetPose)  #创建服务客户端
        response = client()  #发送服务请求
        # print(response)
        return response
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def f_SetPTPCmd(x, y, z, r, isQueued=1):
    """
    功能：设置机械臂的位置
    x: x坐标
    y: y坐标
    z: z坐标
    r: 旋转角度
    isQueued: 是否加入队列
    return: result & queuedCmdIndex
    """
    try:
        rospy.init_node('myDobot')
    except rospy.exceptions.ROSException as e:
        pass
    rospy.wait_for_service('DobotServer/SetPTPCmd')   #指向服务，进行等待连接
    try:
        client = rospy.ServiceProxy('DobotServer/SetPTPCmd',SetPTPCmd)  #创建服务客户端
        response = client(1, x, y, z, r)  #发送服务请求
        # print(response)
        return response
    except rospy.ServiceException as e:
        print("设置机械臂的位置失败: {0}".format(e))



def f_SetEndEffectorSuctionCup(suck, enableCtrl=1, isQueued=1):
    """
    功能：设置机械臂的吸盘
    suck: 吸盘状态 0: 关闭 1: 打开
    enableCtrl: 是否启用控制
    isQueued: 是否加入队列
    return: result & queuedCmdIndex
    """
    try:
        if not rospy.core.is_initialized():
            rospy.init_node('myDobot', anonymous=True)
    except rospy.exceptions.ROSException as e:
        # print("ROS节点已经初始化: {}".format(e))
        # 继续执行，因为节点已经初始化
        pass

    rospy.wait_for_service('DobotServer/SetEndEffectorSuctionCup')   #指向服务，进行等待连接
    try:
        client = rospy.ServiceProxy('DobotServer/SetEndEffectorSuctionCup',SetEndEffectorSuctionCup)  #创建服务客户端
        response = client(enableCtrl, suck, isQueued)  #发送服务请求
        # print(response)
        return response
    except rospy.ServiceException as e:
        print("设置机械臂的吸盘失败: {0}".format(e))


def f_ClearAllAlarmsState():
    """
    功能：清除所有报警状态
    return: result & queuedCmdIndex
    """
    rospy.init_node('myDobot')
    rospy.wait_for_service('DobotServer/ClearAllAlarmsState')   #指向服务，进行等待连接
    try:
        client = rospy.ServiceProxy('DobotServer/ClearAllAlarmsState', ClearAllAlarmsState)  #创建服务客户端
        response = client()  #发送服务请求
        # print(response)
        return response
    except rospy.ServiceException as e:
        print("清除所有报警状态失败: {0}".format(e))


def test_func():
    # print(f_SetHomeCmd())
    # print(f_SetHomeParams())
    # print(f_GetPose())
    # print(f_SetPTPCmd(183.0, -2, 22.4, -0.7))
    # print(f_SetEndEffectorSuctionCup(0))  # 关闭吸盘
    print(f_ClearAllAlarmsState()) # 清除所有报警状态
    # goto_home_and_set_home()
    pass


if __name__ == "__main__":
    test_func()

#!/usr/bin/env python
#-*- coding:utf-8 -*-

"""
功能：获取机器人位姿
该文件包含了获取机器人位姿的函数
- Get_robot_pose(): 获取机器人位置
- Get_current_pose(): 同步获取机器人当前位置
- callback(): 回调函数：处理接收到的位姿数据
- set_goal(): 通过话题/move_base/goal设置目标位置
- cancel_goal(): 取消目标位置
- print_pose(): 打印位置信息
- test_func(): 测试函数：测试api函数
- main(): 主函数：获取机器人位姿

作者：仝铁
创建日期：2025年3月27日
修改日期：2025年3月28日

"""

import rospy
import time
from geometry_msgs.msg import Pose
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalID, GoalStatusArray


def callback(data):
    """
    回调函数：处理接收到的位置数据
    """
    # 打印接收到的位姿数据
    print("接收到位姿数据：")
    print("位置 - x: {}, y: {}, z: {}".format(data.position.x, data.position.y, data.position.z))
    print("方向 - x: {}, y: {}, z: {}, w: {}".format(data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w))
    # 这里可以添加更多的处理逻辑，例如将数据存储到文件或数据库中


def Get_robot_pose():
    """
    功能：获取机器人位置
    """
    # 初始化 ROS 节点
    rospy.init_node('myRobot', anonymous=True)
    
    # 创建一个订阅者，订阅机器人位姿话题
    pub = rospy.Subscriber('robot_pose', Pose, queue_size=10, callback=callback)
    
    # 等待数据到达
    rospy.spin()


def Get_current_pose():
    """
    功能：同步获取机器人当前位置
    返回：Pose 对象
    """
    # 初始化 ROS 节点（如果尚未初始化）
    if not rospy.core.is_initialized():
        rospy.init_node('myRobot', anonymous=True)
    
    # 阻塞直到接收到机器人位姿消息
    pose = rospy.wait_for_message('robot_pose', Pose)
    return pose


def set_goal(x, y, z, x1, y1, z1, w1):
    """
    功能：通过话题/move_base/goal设置目标位置
    参数:
        x, y, z: 目标位置的坐标
        x1, y1, z1, w1: 目标位置的四元数方向
    """
    # 初始化ROS节点（如果尚未初始化）
    if not rospy.core.is_initialized():
        rospy.init_node('myRobot', anonymous=True)
    
    # 创建发布者，发布到/move_base/goal话题，使用正确的消息类型
    pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)
    
    # 创建MoveBaseActionGoal消息
    goal_msg = MoveBaseActionGoal()
    
    # 设置消息头
    goal_msg.header.stamp = rospy.Time.now()
    goal_msg.header.frame_id = "map"
    
    # 设置目标ID
    goal_msg.goal_id = GoalID()
    goal_msg.goal_id.stamp = rospy.Time.now()
    goal_msg.goal_id.id = "nav_goal_" + str(rospy.Time.now().to_sec())
    
    # 设置目标位置
    goal_msg.goal.target_pose.header.stamp = rospy.Time.now()
    goal_msg.goal.target_pose.header.frame_id = "map"
    
    goal_msg.goal.target_pose.pose.position.x = x
    goal_msg.goal.target_pose.pose.position.y = y
    goal_msg.goal.target_pose.pose.position.z = z
    
    goal_msg.goal.target_pose.pose.orientation.x = x1
    goal_msg.goal.target_pose.pose.orientation.y = y1
    goal_msg.goal.target_pose.pose.orientation.z = z1
    goal_msg.goal.target_pose.pose.orientation.w = w1
    
    # 等待发布者连接
    rospy.sleep(1.0)
    
    # 发布消息
    pub.publish(goal_msg)
    rospy.loginfo("已发送目标点: x: {}, y: {}, z: {}".format(x, y, z))
    
    return True


def cancel_goal():
    """
    功能：取消目标位置
    """
    # 初始化ROS节点（如果尚未初始化）
    if not rospy.core.is_initialized():
        rospy.init_node('myRobot', anonymous=True)
    
    # 创建发布者，发布到/move_base/cancel话题
    pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
    
    # 创建GoalID消息（空消息会取消所有目标）
    cancel_msg = GoalID()
    
    # 等待发布者连接
    rospy.sleep(1.0)
    
    # 发布消息
    pub.publish(cancel_msg)
    rospy.loginfo("已取消所有导航目标")
    
    return True


def get_status():
    """
    功能：获取导航状态

    返回：状态码和状态文本
    """
    try:
        # 初始化ROS节点（如果尚未初始化）
        if not rospy.core.is_initialized():
            rospy.init_node('myRobot', anonymous=True)
    except rospy.exceptions.ROSException as e:
        # 如果节点已经初始化，继续执行
        pass
    
    # 等待数据到达
    status_msg = rospy.wait_for_message('/move_base/status', GoalStatusArray, timeout=1)
    
    # 打印导航状态
    # print("导航状态: {}".format(status_msg.status_list[-1].status))
    status_text = {
        0: "等待中",
        1: "正在执行",
        2: "已完成",
        3: "被抢占",
        4: "被取消",
        5: "被拒绝"
    }.get(status_msg.status_list[-1].status, "未知状态({})".format(status_msg.status_list[-1].status))
    # print("导航状态文本: {}".format(status_text))
    
    return status_msg.status_list[-1].status, status_text


def navigate_to_position(position_data):
    """
    导航到指定位置
    """
    if not position_data:
        print("无效的位置数据")
        return False
    
    x = position_data.get('x', 0)
    y = position_data.get('y', 0)
    z = position_data.get('z', 0)
    
    # 方向四元数
    orientation_x = position_data.get('orientation_x', 0)
    orientation_y = position_data.get('orientation_y', 0)
    orientation_z = position_data.get('orientation_z', 0)
    orientation_w = position_data.get('orientation_w', 1)  # 默认为1，表示无旋转
    
    print("正在导航到位置 x:{}, y:{}, z:{}".format(x, y, z))
    
    # 调用Nav_Api中的set_goal函数设置导航目标
    result = set_goal(x, y, z, orientation_x, orientation_y, orientation_z, orientation_w)
    
    if result:
        print("导航目标已设置，等待到达...")
        # 等待导航完成
        while True:
            # 检查导航状态
            status, status_text = get_status()
            if status == 3:
                print("导航完成，状态: {}".format(status_text))
                break
            elif status == 4:
                print("导航被取消，状态: {}".format(status_text))
                return False
            elif status == 5:
                print("导航被拒绝，状态: {}".format(status_text))
                return False
            elif status == 2:
                print("导航已完成，状态: {}".format(status_text))
                break
            elif status == 1:
                print("导航正在执行，状态: {}".format(status_text))
                time.sleep(0.2)  # 等待0.2秒后再次检查状态
                # 等待0.2秒后再次检查状态
                continue
            else:
                print("导航未完成，状态: {}".format(status_text))
                return False
        
        return True
    else:
        print("导航目标设置失败")
        return False
    

def set_xy_goal_tolerance(xy_goal_tolerance=0.005):
    """
    功能：设置xy目标容差
    参数:
        xy_goal_tolerance: xy目标容差（单位：米）
    返回:
        bool: 设置成功返回True，失败返回False
    """
    try:
        # 初始化ROS节点（如果尚未初始化）
        if not rospy.core.is_initialized():
            rospy.init_node('myRobot', anonymous=True)
        
        # 设置参数
        param_name = "/move_base/TebLocalPlannerROS/xy_goal_tolerance"
        rospy.set_param(param_name, xy_goal_tolerance)
        
        # 验证参数是否设置成功
        new_value = rospy.get_param(param_name)
        if abs(new_value - xy_goal_tolerance) < 1e-6:  # 浮点数比较
            rospy.loginfo("xy目标容差已设置为: %.3f", xy_goal_tolerance)
            return True
        else:
            rospy.logwarn("xy目标容差设置失败! 期望: %.3f, 实际: %.3f", xy_goal_tolerance, new_value)
            return False
    except Exception as e:
        rospy.logerr("设置xy目标容差时出错: %s", str(e))
        return False


def set_yaw_goal_tolerance(yaw_goal_tolerance=0.015):
    """
    功能：设置yaw目标容差
    参数:
        yaw_goal_tolerance: yaw目标容差（单位：弧度）
    返回:
        bool: 设置成功返回True，失败返回False
    """
    try:
        # 初始化ROS节点（如果尚未初始化）
        if not rospy.core.is_initialized():
            rospy.init_node('myRobot', anonymous=True)
        
        # 设置参数
        param_name = "/move_base/TebLocalPlannerROS/yaw_goal_tolerance"
        rospy.set_param(param_name, yaw_goal_tolerance)
        
        # 验证参数是否设置成功
        new_value = rospy.get_param(param_name)
        if abs(new_value - yaw_goal_tolerance) < 1e-6:  # 浮点数比较
            rospy.loginfo("yaw目标容差已设置为: %.3f", yaw_goal_tolerance)
            return True
        else:
            rospy.logwarn("yaw目标容差设置失败! 期望: %.3f, 实际: %.3f", yaw_goal_tolerance, new_value)
            return False
    except Exception as e:
        rospy.logerr("设置yaw目标容差时出错: %s", str(e))
        return False


def print_pose(pose, debug=False):
    """
    功能：仅打印robot位置信息
    """
    if debug:
        print(pose)
        return
    print("位置 - x: {}, y: {}, z: {}".format(pose.position.x, pose.position.y, pose.position.z))
    print("方向 - x: {}, y: {}, z: {}, w: {}".format(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))


def test_func():
    """
    测试函数：测试api函数
    """
    orgin_pose = Get_current_pose()  # 获取当前位姿
    print_pose(orgin_pose)  # 打印当前位姿

    print("等待8秒...")
    time.sleep(8)

    current_pose = Get_current_pose()  # 获取当前位姿
    print_pose(current_pose)  # 打印当前位姿

    # 设置目标位置
    set_goal(orgin_pose.position.x + 0.1, orgin_pose.position.y + 0.1, orgin_pose.position.z, orgin_pose.orientation.x, orgin_pose.orientation.y, orgin_pose.orientation.z, orgin_pose.orientation.w)
    print("目标位置已设置")
    cancel_goal()  # 取消目标位置



def test_func2():
    # get_status()  # 获取导航状态
    cancel_goal()  # 取消目标位置


if __name__ == '__main__':
    # test_func()
    test_func2()

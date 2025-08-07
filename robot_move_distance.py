#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
from geometry_msgs.msg import Twist, Point
from math import sqrt, pow
import tf

class SimpleForward():
    def __init__(self):
        try:
            # 初始化ROS节点
            rospy.init_node('simple_forward', anonymous=False)
        except rospy.ROSException as e:
            # rospy.logerr("ROS节点初始化失败: %s", e)
            pass
        
        # 参数设置
        self.rate = rospy.Rate(20)
        self.speed = 0.2  # 前进速度
        self.tolerance = 0.05  # 容错范围
        
        # 发布速度命令
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        
        # 设置坐标系
        self.base_frame = '/base_footprint'
        self.odom_frame = '/odom_combined'
        
        # 初始化TF监听器
        self.tf_listener = tf.TransformListener()
        rospy.sleep(1)  # 等待TF缓冲区填充
        
        # 确保可以看到odom和base帧
        self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(5.0))
    
    def move_distance(self, distance):
        """
        让机器人前进指定距离
        
        参数:
            distance (float): 要前进的距离，单位为米
        """
        rospy.loginfo("开始前进 %.2f 米...", distance)
        
        # 获取起始位置
        self.position = self.get_position()
        x_start = self.position.x
        y_start = self.position.y
        
        move_cmd = Twist()
        
        while not rospy.is_shutdown():
            # 获取当前位置
            self.position = self.get_position()
            
            # 计算已经移动的距离
            current_distance = sqrt(pow((self.position.x - x_start), 2) + pow((self.position.y - y_start), 2))
            
            # 计算与目标距离的差值
            error = current_distance - abs(distance)
            
            # 判断是否达到目标
            if abs(error) < self.tolerance:
                rospy.loginfo("到达目标位置!")
                break
            
            # 设置移动速度（考虑距离正负值决定前进或后退）
            direction = 1 if distance > 0 else -1
            move_cmd.linear.x = direction * self.speed if error < 0 else -direction * self.speed
            
            # 发布速度命令
            self.cmd_vel.publish(move_cmd)
            self.rate.sleep()
        
        # 停止机器人
        self.cmd_vel.publish(Twist())
        rospy.loginfo("移动完成")
        return True
    
    def get_position(self):
        # 获取当前位置
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            return Point(*trans)
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF异常")
            return Point()
    
    def shutdown(self):
        # 关闭节点时停止机器人
        rospy.loginfo("停止机器人...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        robot = SimpleForward()
        # 演示：前进1米
        robot.move_distance(0.08)
        # robot.move_distance(0.08)
        # robot.move_distance(0.08)
        # 可以继续调用，例如后退0.5米
        # robot.move_distance(-0.5)
        rospy.loginfo("任务完成")
    except rospy.ROSInterruptException:
        rospy.loginfo("程序终止")

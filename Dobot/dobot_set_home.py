#!/usr/bin/env python
#-*- coding:utf-8 -*-

"""
Dobot机械臂返回零点并设置零点

作者：仝铁
创建日期：2025年3月30日
修改日期：2025年3月30日

"""

from Dobot_Api import goto_home_and_set_home


def main():
    """
    执行设置零点操作
    """
    goto_home_and_set_home(sleep_time=5)
    print("\n########## 已设置零点 ##########\n")


if __name__ == "__main__":
    main()

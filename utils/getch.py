#!/usr/bin/env python
# -*- coding:utf-8 -*-

"""
功能：读取单个字符输入（无需回车）

作者：仝铁
创建日期：2025年3月27日

"""

import sys
import tty
import termios


def getch():
    """
    获取单个字符输入（无需回车）
    
    返回:
    - 输入的字符
    """
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""

本程序用于仿真，直接调用gazebo服务实现
提供ROS服务，可获得激光基准面姿态
并更新tf（相对base_link）到参数服务器，laser_trans，laser_rot

"""
from ccd_laser.srv import GetLaserPos
from gazebo_msgs.srv import GetLinkState
import rospy
import tf
import geometry_msgs.msg
import time
import math
import numpy as np
import matplotlib.pyplot as plt
import random#随机数生成


#服务回调函数
def handle_get_laser_plane(req):
    print "====激光基准获取服务被调用===="
    #初始化tf发布者
    broadcaster = tf.TransformBroadcaster()
    #初始化tf监听者
    listener = tf.TransformListener()
    """
    通过'/gazebo/get_link_state'的服务来查询link状态
    #用来模拟ccd测量laser平面，相对base_link坐标系
    #y，z与effector相等
    #x和姿态与wall相等，姿态需要转换方向
    """
    #获得wall相对base_link
    rospy.wait_for_service('/gazebo/get_link_state')
    try:
        get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        wall = get_link_state('Wall_19', 'base_link')
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    #获得effector相对base_link
    try:
        get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        effector = get_link_state('effector_base', 'base_link')
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    #分离wall各参数
    wall_trans = wall.link_state.pose.position
    wall_rot = wall.link_state.pose.orientation

    #分离effector各参数
    effector_trans = effector.link_state.pose.position
    effector_rot = effector.link_state.pose.orientation

    #yz与末端相同，x为墙前0.3m，垂直墙面方向加上+/-1mm误差
    trans = [wall_trans.x-0.3+random.uniform(-0.001,0.001),effector_trans.y,effector_trans.z]
    euler_temp = tf.transformations.euler_from_quaternion([wall_rot.x,wall_rot.y,wall_rot.z,wall_rot.w])
    #姿态与墙面相同
    rot = tf.transformations.quaternion_from_euler(euler_temp[0],euler_temp[1],euler_temp[2]+(math.pi/2)).tolist()
    print trans,rot
    #保存到参数服务器
    rospy.set_param('laser_trans',trans)
    rospy.set_param('laser_rot',rot)
    return True


#服务主函数
def ccd_get_plane_server():
    #初始化ros节点
    rospy.init_node('ccd_get_plane_server')
    #初始化服务（服务名，数据类型，回调函数）
    s = rospy.Service('ccd_get_plane', GetLaserPos, handle_get_laser_plane)
    #初始化tf发布节点，http://docs.ros.org/jade/api/tf/html/python/tf_python.html#tf.Transformer.allFramesAsString
    print "====等待调用 激光基准获取服务===="
    rospy.spin()

if __name__ == "__main__":
    try:
       ccd_get_plane_server()       
    except rospy.ROSInterruptException:
        pass                 
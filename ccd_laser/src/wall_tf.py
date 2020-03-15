#!/usr/bin/env python 
# -*- coding: utf-8 -*-
#程序功能：将参数服务器上读取的坐标信息发布成TF
import rospy
import tf
import geometry_msgs.msg

if __name__ == '__main__':
    #初始化ros节点
    rospy.init_node('wall_tf_broadcaster')

    #==============发布静态tf================
    #初始化tf发布节点，http://docs.ros.org/jade/api/tf/html/python/tf_python.html#tf.Transformer.allFramesAsString
    broadcaster = tf.TransformBroadcaster()
    #设置发布频率，一般为10Hz
    rate = rospy.Rate(10)
    #循环检测，如果没有关闭则一直检测
    while not rospy.is_shutdown():      
        #未读取到数据->等待
        if  rospy.has_param('wall_trans'):
            #从参数服务器读取墙面tf
            wall_trans = rospy.get_param("wall_trans")
            wall_rot = rospy.get_param("wall_rot")
            #发布相应tf（位置，姿态，时间戳，子坐标系，父坐标系）
            #broadcaster.sendTransform(position,orientation,rospy.Time.now(),"wall","kinect_frame_optical")
            broadcaster.sendTransform(wall_trans,wall_rot,rospy.Time.now(),"wall","base_link")
            print "wall: position=",wall_trans," orientation=",wall_rot
        rate.sleep()
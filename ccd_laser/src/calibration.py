#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""

提供ROS服务，可获得激光基准面姿态,返回面方程系数，
并更新tf（相对base_link）到参数服务器，laser_trans，laser_rot

"""
from ccd_laser.srv import GetLaserPos
import rospy
import tf
import geometry_msgs.msg
import serial
import time
import math
import numpy as np
import matplotlib.pyplot as plt
import binascii#str转16进制


#修改为3个ccd对应的串口名
s0 = '/dev/ttyUSB0'
s1 = '/dev/ttyUSB2'
s2 = '/dev/ttyUSB1'


#传入串口对象，读取所有像素的值，通过串口读取CCD数据，返回最小值（光强最大）的位置
def ccd_getposition(ser):#传入对应串口对象
    for i in range(1):  #循环，过滤无效值   
        #写入转换命令@c0080#@
        result=ser.write("@c0120#@".encode("utf-8"))
        print "指令发送成功，一共发送",result,"位||等待接收串口数据..........."
        time.sleep(0.2)#等待时间应该大于积分时间
        
        time_start = time.time()
        a = binascii.b2a_hex(ser.read(7296))#读取十六进制数据14592(3648*2)
        time_end = time.time()
        print '读取串口用时:', time_end-time_start, 's'
                 
        #将十六进制数据分离并转换为十进制
        b = []
        for i in range(0,14592,4):
            b.append(int(a[i:i+4],16))
        
        #过滤无效数据，通过最小值判断
        if min(b) < 4000:#设置过滤的阈值
            x = np.arange(0,3648,dtype = float)*0.008
            print x[b.index(min(b))]
            if 1:      #是否绘图
                plt.rcParams['figure.dpi'] = 100
                #plt.cla()#清除
                plt.xlabel("Position(mm)")
                plt.ylabel("Value")
                plt.title("CCD test")
                plt.plot(x,b)
                plt.show()
            #np.save("ccd_test2.npy",b)
            return x[b.index(min(b))]
            break
  
    
#函数功能：方法二测量，读取所有像素点的光强值
#传入串口名，初始化串口，并计算最小值位置
def ccd_getposition2(s):
    #打开串口
    ser = serial.Serial(s,115200 ,timeout=2)
    #print "串口参数：",serial
    #判断串口是否打开成功
    if ser.isOpen(): 
        print "串口打开成功！！！" 
    else: 
        print "串口打开失败！！！" 
    #调用函数，读取最小值
    pos = ccd_getposition(ser)
    print '峰值点位于：',pos,'mm'
    ser.close()#关闭串口
    return pos


#函数功能：方法3测量位置，返回两个最小值点和值
#传入串口名,返回最小值位置,下位机直接计算位置
def ccd_getposition3(s):
    #打开串口
    ser = serial.Serial(s,115200,timeout=1)
    #print "串口参数：",serial
    #判断串口是否打开成功
    if ser.isOpen(): 
        print "串口打开成功！！！"
    else: 
        print "串口打开失败！！！"

    #写入转换命令@d0080#@,返回类型（位置1 值1 位置2 值2）共4*4=16个16进制数
    result=ser.write("@d0080#@".encode("utf-8"))
    print "指令发送成功，一共发送",result,"位||等待接收串口数据..........."
    time.sleep(0.1)#等待
    time_start = time.time()#记录开始时间
    a = binascii.b2a_hex(ser.read(8))#读取十六进制数据14592(3648*4)
    time_end = time.time()
    print '读取串口用时:', time_end-time_start, 's'
             
    #读取两个直接获取到的位置，将十六进制数据分离并转换为十进制，取均值
    pos=(int(a[0:4],16)+int(a[8:12],16))/2*0.008
    value = int(a[4:8],16)

    print '峰值点位于：',pos,'mm','值：',value
    ser.close()#关闭串口
    return pos,value


#函数功能：方法4测量位置，下位机循环5次
#传入串口名,返回最小值位置,下位机测量连续测量5次的结果
def ccd_getposition4(s):
    #打开串口,下位机处理较慢，不设超时时间
    ser = serial.Serial(s,115200)
    #print "串口参数：",serial
    #判断串口是否打开成功
    if ser.isOpen(): 
        print "串口打开成功！！！" 
    else: 
        print "串口打开失败！！！"

    #写入转换命令@e0080#@,返回类型（位置1 值1 位置2 值2）共4*4=16个16进制数
    result=ser.write("@e0080#@".encode("utf-8"))
    print "指令发送成功，一共发送",result,"位||等待接收串口数据..........."
    time.sleep(0.2)#等待时间应该大于积分时间
    time_start = time.time()#记录开始时间
    a = binascii.b2a_hex(ser.read(10))#读取十六进制数据
    time_end = time.time()
    print '读取串口用时:', time_end-time_start, 's'       
    #读取两个直接获取到的位置，将十六进制数据分离并转换为十进制，取均值
    pos=(int(a[0:4],16)+int(a[4:8],16)+int(a[8:12],16)+int(a[12:16],16)+int(a[16:20],16))/5*0.008
    print '峰值点位于：',pos,'mm'
    ser.close()#关闭串口
    return pos


#函数功能：方法3并行测量3个ccd位置，能够返回值峰值及其光强
#传入串口名,返回最小值位置,下位机测量连续测量5次的结果
def ccd_getposition3_3p(s1,s2,s3):
    #打开串口，下位机处理慢，不设超时时间
    ser1 = serial.Serial(s1,115200)
    ser2 = serial.Serial(s2,115200)
    ser3 = serial.Serial(s3,115200)
    #print "串口参数：",serial
    #判断串口是否打开成功
    if ser1.isOpen() & ser2.isOpen() & ser3.isOpen(): 
        print "串口打开成功！！！" 
    else: 
        print "串口打开失败！！！"

    #写入转换命令@d0080#@,返回类型（位置1 值1 位置2 值2）共4*4=16个16进制数
    result1=ser1.write("@d0080#@".encode("utf-8"))
    result2=ser2.write("@d0080#@".encode("utf-8"))
    result3=ser3.write("@d0080#@".encode("utf-8"))
    print "指令发送成功||等待接收串口数据..........."
    time.sleep(0.1)#等待时间应该大于积分时间
    time_start = time.time()#记录开始时间
    a1 = binascii.b2a_hex(ser1.read(8))#读取十六进制数据
    a2 = binascii.b2a_hex(ser2.read(8))#读取十六进制数据
    a3 = binascii.b2a_hex(ser3.read(8))#读取十六进制数据
    time_end = time.time()
    print '读取串口用时:', time_end-time_start, 's'       
    #读取两个直接获取到的位置，将十六进制数据分离并转换为十进制，取均值
    pos1=(int(a1[0:4],16)+int(a1[8:12],16))/2*0.008
    pos2=(int(a2[0:4],16)+int(a2[8:12],16))/2*0.008
    pos3=(int(a3[0:4],16)+int(a3[8:12],16))/2*0.008
    value1 = int(a1[4:8],16)
    value2 = int(a2[4:8],16)
    value3 = int(a3[4:8],16)
    print '峰值点1位于：',pos1,'mm'
    print '峰值点2位于：',pos2,'mm'
    print '峰值点3位于：',pos3,'mm'
    ser1.close()#关闭串口
    ser2.close()
    ser3.close()
    return [pos1,pos2,pos3],[value1,value2,value3]


#函数功能：方法4并行测量3个ccd位置
#传入串口名,返回最小值位置,下位机测量连续测量5次的结果
def ccd_getposition4_3p(s1,s2,s3):
    #打开串口，下位机处理慢，不设超时时间
    ser1 = serial.Serial(s1,115200)
    ser2 = serial.Serial(s2,115200)
    ser3 = serial.Serial(s3,115200)
    #print "串口参数：",serial
    #判断串口是否打开成功
    if ser1.isOpen() & ser2.isOpen() & ser3.isOpen(): 
        print "串口打开成功！！！" 
    else: 
        print "串口打开失败！！！"

    #写入转换命令@e0080#@,返回类型（位置1 值1 位置2 值2）共4*4=16个16进制数
    result1=ser1.write("@e0080#@".encode("utf-8"))
    result2=ser2.write("@e0080#@".encode("utf-8"))
    result3=ser3.write("@e0080#@".encode("utf-8"))
    print "指令发送成功||等待接收串口数据..........."
    time.sleep(0.2)#等待时间应该大于积分时间
    time_start = time.time()#记录开始时间
    a1 = binascii.b2a_hex(ser1.read(10))#读取十六进制数据
    a2 = binascii.b2a_hex(ser2.read(10))#读取十六进制数据
    a3 = binascii.b2a_hex(ser3.read(10))#读取十六进制数据
    time_end = time.time()
    print '读取串口用时:', time_end-time_start, 's' 
    #将十六进制数据分离并转换为十进制
    b1 = []
    b2 = []
    b3 = []
    for i in range(0,20,4):
        b1.append(int(a1[i:i+4],16))      
        b2.append(int(a2[i:i+4],16))
        b3.append(int(a3[i:i+4],16))
    
    #读取两个直接获取到的位置，将十六进制数据分离并转换为十进制，取均值
    pos1=np.mean(b1)*0.008
    pos2=np.mean(b2)*0.008
    pos3=np.mean(b3)*0.008
    print '峰值点1位于：',pos1,'mm'
    print '峰值点2位于：',pos2,'mm'
    print '峰值点3位于：',pos3,'mm'
    ser1.close()#关闭串口
    ser2.close()
    ser3.close()
    return pos1,pos2,pos3


#方法三测量位置，测量t次，求平均
def ccd_getposition3_3p_ts(s1,s2,s3,t = 20):
    p1 = []
    p2 = []
    p3 = []
    for i in range(t):#循环测量t次
        p = ccd_getposition3_3p(s1,s2,s3)[0]
        print p
        p1.append(p[0])
        p2.append(p[1])
        p3.append(p[2])
    p1.sort()#排序
    p2.sort()
    p3.sort()
    result = [np.mean(p1[5:t-5]),np.mean(p2[5:t-5]),np.mean(p3[5:t-5])]#去极值，求平均
    print '测量',t,'次后，去除最大和最小后平均，结果为：',result
    return result


#求两向量夹角，v1，v2格式[x,y,z]
def get_angle_by_vectors(v1, v2):
    #转换为numpy数组
    a = np.array(v1)
    b = np.array(v2)
    #求向量长度
    La = np.sqrt(a.dot(a))
    Lb = np.sqrt(b.dot(b))
    #求cos值
    cos_angle = a.dot(b)/(La*Lb)
    #求角度（弧度）
    angle = np.arccos(cos_angle)
    #转为角度制
    return angle


#函数功能已知3点坐标，求平面ax+by+cz+d=0;
def get_panel_by_3points(p1,p2,p3):
    a = (p2[1] - p1[1])*(p3[2] - p1[2]) - (p2[2] - p1[2])*(p3[1] - p1[1])
    b = (p2[2] - p1[2])*(p3[0] - p1[0]) - (p2[0] - p1[0])*(p3[2] - p1[2])
    c = (p2[0] - p1[0])*(p3[1] - p1[1]) - (p2[1] - p1[1])*(p3[0] - p1[0])
    if(a < 0):
        a = -a
        b = -b
        c = -c
    d = 0 - (a * p1[0] + b*p1[1] + c*p1[2]) 
    #print '平面方程:(%f)X+(%f)Y+(%f)Z+(%f)=0' %(a,b,c,d)
    #print '平面中(x,y)=(0,0)点的z坐标：{}'.format(- d/c)
    #print '与原始平面夹角：',get_angle_by_vectors([a,b,c],[1,0,0])*180/np.pi,'度'
    return a,b,c,d


#函数功能:传入ccd串口名，求平面ax+by+cz+d=0,返回参数abcd;
def get_panel_by_ccd(s1,s2,s3):
    #串口读取传感器位置
    ccd_pos = ccd_getposition3_3p_ts(s1,s2,s3,10)#求10次来滤波，提高速度===================================================
    #根据一直模型得到3个点============================================修改标定参数======================================
    p1 = [ccd_pos[0]-14.5,37.5,0]
    p2 = [ccd_pos[1]-14.65,0,30]
    p3 = [ccd_pos[2]-14.8,-37.5,0]
    #转换为单位m
    for i in range(3):
        p1[i] *= 0.001
        p2[i] *= 0.001
        p3[i] *= 0.001
    #调用函数，通过三点求面方程
    print p1,p2,p3
    a,b,c,d = get_panel_by_3points(p1,p2,p3)
    print '平面方程:(%f)X+(%f)Y+(%f)Z+(%f)=0' %(a,b,c,d)
    print '平面中(y,z)=(0,0)点的z坐标：{}'.format(- d/a)
    print '与原始平面夹角：',get_angle_by_vectors([a,b,c],[1,0,0])*180/np.pi,'度'
    return a,b,c,d


#通过面方程（参数abcd）求解TF所需要的位姿参数
def get_tf_from_plane(a,b,c,d):
    position = [-d/a,0,0]
    #求转角
    angle = get_angle_by_vectors([1,0,0],[a,b,c])
    #求转轴，向量叉乘：(x1,y1,z1)X(x2,y2,z2)=(y1z2-y2z1, z1x2-z2y1, x1y2-x2y1)
    ax = [0,-c,b]#叉乘结果
    #单位向量
    if( (b == 0) & (c == 0)):
        ax = [1,0,0]
    else:
        ax = [i/math.sqrt(b*b+c*c) for i in ax]#归一化
    w = math.cos(angle/2)
    x = math.sin(angle/2)*ax[0]
    y = math.sin(angle/2)*ax[1]
    z = math.sin(angle/2)*ax[2]
    orientation = [x,y,z,w]
    print '求得平面tf，位置：',position,'四元数：',orientation
    return position,orientation


#服务回调函数
def handle_get_laser_plane(req):
    print "====激光基准获取服务被调用===="
    #初始化tf发布者
    broadcaster = tf.TransformBroadcaster()
    #初始化tf监听者
    listener = tf.TransformListener()
    #获取激光面tf，相对于ccd，并短暂发布
    ccd_pos = get_panel_by_ccd(s0,s1,s2)
    trans0,rot0 = get_tf_from_plane(ccd_pos[0],ccd_pos[1],ccd_pos[2],ccd_pos[3])
    #设置发布频率，一般为10Hz
    rate = rospy.Rate(10)
    for i in range(5):  
        broadcaster.sendTransform(trans0,rot0,rospy.Time.now(),"laser_plane_tmp","ccd_link")
        rate.sleep()
    #获取激光面tf，相对于base_link,保存到参数服务器
    (trans,rot) = listener.lookupTransform('base_link','laser_plane_tmp',rospy.Time(0))
    rospy.set_param('laser_trans',trans)
    rospy.set_param('laser_rot',rot)
    return [ccd_pos[0],ccd_pos[1],ccd_pos[2],ccd_pos[3]]


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
       ccd_getposition3_3p_ts(s0,s1,s2,50)        
    except rospy.ROSInterruptException:
        pass                 
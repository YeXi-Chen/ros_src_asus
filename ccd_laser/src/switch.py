#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""

读取行程开关状态（按下5代表碰撞）
发布状态到topic和设置param

"""

import time
import rospy
from std_msgs.msg import Bool
import moveit_commander
import sys, select, termios, tty


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def talker():
    #初始化ros节点和发布者
    pub = rospy.Publisher('switch_state', Bool, queue_size=1)
    rospy.init_node('switch_state_pub', anonymous=True)

    # 初始化move_group的API
    moveit_commander.roscpp_initialize(sys.argv)
    # 初始化需要使用move group控制的机械臂中的arm group
    arm = moveit_commander.MoveGroupCommander('ur')


    state =0

    while not rospy.is_shutdown():
        key = getKey()#检测按键
        if key == '5':
            print "检测到碰撞，机械臂急停！！！！"
            state = 1
            arm.stop()
            time.sleep(2)
        elif (key == '\x03'):
            break
        else：
            state = 0
        print "行程开关状态:",state,"key:",key
        #print time.time()
        #print rospy.get_param('seitch_state')
        rospy.set_param('switch_state',state)
        pub.publish(state)

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    try:
        talker()
    except rospy.ROSInterruptException:
        pass                 
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)



 


 
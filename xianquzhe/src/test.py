#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
moveit正运动学
控制底盘移动
'''

import rospy, sys
import moveit_commander
import move

if __name__ == "__main__":
    try:
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        # 初始化ROS节点
        rospy.init_node('move_fk_node', anonymous=True)

        #生成对象实例
        move = move.RotFk()
        move.run(0,0)
        print move.get_pose()
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
    except rospy.ROSInterruptException:
        pass
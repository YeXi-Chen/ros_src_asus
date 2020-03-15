#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
moveit正运动学
控制底盘移动
'''

import rospy, sys
import moveit_commander

class MoveFk(object):
    """moveit正运动学
    控制底盘移动：.run（x,y,r）
    """
    def __init__(self):
        # 初始化需要使用move group控制的机械臂
        self.arm = moveit_commander.MoveGroupCommander("move")
        
        # 设置机械臂运动的允许误差值
        self.arm.set_goal_joint_tolerance(0.001)

        # 设置允许的最大速度和加速度
        self.arm.set_max_acceleration_scaling_factor(0.5)
        self.arm.set_max_velocity_scaling_factor(0.5)
    
    def run(self,x ,y ,r = 0):#位置x，y,z向转交
        # 设置机械臂的目标位置，使用六轴的位置数据进行描述（单位：弧度）
        joint_positions = [x, y, r]
        self.arm.set_joint_value_target(joint_positions)       
        # 控制机械臂完成运动
        self.arm.go()
        return

    def get_pose(self):
        return self.arm.get_current_pose().pose


class RotFk(object):
    """moveit正运动学
    控制升降和转角：.run（h,r）
    """
    def __init__(self):
        # 初始化需要使用move group控制的机械臂
        self.arm = moveit_commander.MoveGroupCommander("rot")
        
        # 设置机械臂运动的允许误差值
        self.arm.set_goal_joint_tolerance(0.001)

        # 设置允许的最大速度和加速度
        self.arm.set_max_acceleration_scaling_factor(0.5)
        self.arm.set_max_velocity_scaling_factor(0.5)
    
    def run(self,h ,r = 0):#传入高度和转角
        # 设置机械臂的目标位置，使用六轴的位置数据进行描述（单位：弧度）
        joint_positions = [h, r]
        self.arm.set_joint_value_target(joint_positions)       
        # 控制机械臂完成运动
        self.arm.go()
        return

    def get_pose(self):
        return self.arm.get_current_pose().pose

'''
if __name__ == "__main__":
    try:
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        # 初始化ROS节点
        rospy.init_node('move_fk_node', anonymous=True)

        #生成对象实例
        move = MoveFk()
        move.run(0,0,0)
        print move.get_pose()
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
    except rospy.ROSInterruptException:
        pass
'''
#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
moveit正运动学
控制底盘移动
'''

import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose

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

class ArmIk(object):
    """机械臂真逆运动学
    """
    def __init__(self):
        # 初始化需要使用move group控制的机械臂
        self.arm = moveit_commander.MoveGroupCommander("ur")

        # 当运动规划失败后，允许重新规划
        self.arm.allow_replanning(True)
        self.arm.set_planning_time(10)
        
        # 设置机械臂运动的允许误差值
        self.arm.set_goal_joint_tolerance(0.001)
        self.arm.set_goal_position_tolerance(0.001)
        self.arm.set_goal_orientation_tolerance(0.01)

        # 设置允许的最大速度和加速度
        self.arm.set_max_acceleration_scaling_factor(0.5)
        self.arm.set_max_velocity_scaling_factor(0.5)

        # 设置目标位置所使用的参考坐标系
        self.reference_frame = 'base_link'
        self.arm.set_pose_reference_frame(self.reference_frame)
        # 获取终端link的名称
        self.end_effector_link = self.arm.get_end_effector_link()
        
    def run_joint(self,ur1,ur2,ur3,ur4,ur5,ur6):
        '''传入关节6维数组，控制运动,单位弧度
        '''
        joint_positions = [ur1,ur2,ur3,ur4,ur5,ur6]
        self.arm.set_joint_value_target(joint_positions)       
        # 控制机械臂完成运动
        self.arm.go()
        return

    def run_pose(self,pose):
        '''传入末端位姿，控制运动,单位弧度
        传入类型geometry_msgs/Pose
        示例：
        target_pose = move.get_pose()
        target_pose.position.z += 0.1
        move.run_pose(target_pose)
        '''
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose = pose     

        # 设置机器臂当前的状态作为运动初始状态
        self.arm.set_start_state_to_current_state()    
        # 设置机械臂终端运动的目标位姿
        self.arm.set_pose_target(target_pose,self.end_effector_link)
        # 规划运动路径
        traj = self.arm.plan()
        # 按照规划的运动路径控制机械臂运动
        self.arm.execute(traj)
        return

    def get_pose(self):
        return self.arm.get_current_pose().pose

    def get_joint(self):
        return self.arm.get_current_joint_values()

if __name__ == "__main__":
    try:
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        # 初始化ROS节点
        rospy.init_node('move_fk_node', anonymous=True)

        #生成对象实例
        move = ArmIk()
        target_pose = move.get_pose()
        target_pose.position.z += 0.1

        move.run_pose(target_pose)
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
    except rospy.ROSInterruptException:
        pass
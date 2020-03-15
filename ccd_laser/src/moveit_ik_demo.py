#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''程序介绍
demo：
逆运动学
moveit中，给定机械臂关节角度，控制机械臂

'''
import rospy, sys
import moveit_commander
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene
from copy import deepcopy
import math

#传入ccd的位置，类型geometry_msgs.msg中的Pose(坐标和转角rad)
#返回转换后UR5末端的位姿
def ccd_to_ur5_ik(x,y,z,a):
    ur5_pose = Pose()
    ur5_pose.position.x = x - 0.079
    ur5_pose.position.y = y + 0.05*math.sin(a)
    ur5_pose.position.z = z - 0.05*math.cos(a)
    ur5_pose.orientation.x = -math.sin((math.pi/2+a)/2)*math.sin(-math.pi/2/2)
    ur5_pose.orientation.y = math.sin((math.pi/2+a)/2)*math.cos(-math.pi/2/2)
    ur5_pose.orientation.z = math.cos((math.pi/2+a)/2)*math.sin(-math.pi/2/2)
    ur5_pose.orientation.w = math.cos((math.pi/2+a)/2)*math.cos(-math.pi/2/2)
    return ur5_pose

class MoveItIkDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('moveit_ik_demo')

        # 初始化场景对象
        scene = PlanningSceneInterface()
        rospy.sleep(1)
                
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = moveit_commander.MoveGroupCommander('manipulator')
                
        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()
                        
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'wall'
        arm.set_pose_reference_frame(reference_frame)
                
        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
        arm.set_planning_time(10)
        
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.002)
        arm.set_goal_orientation_tolerance(0.01)
        
    # ===========移除场景中之前运行残留的物体================
        scene.remove_attached_object(end_effector_link, 'tool')
        scene.remove_world_object('wall') 
        scene.remove_world_object('floor') 
        scene.remove_world_object('target')

    #===================在末端添加物体，防止碰撞=============
        # 设置tool的三维尺寸
        tool_size = [0.01, 0.12, 0.07]
        
        # 设置tool的位姿
        p = PoseStamped()
        p.header.frame_id = end_effector_link
        p.pose.position.x = -0.05
        p.pose.position.y = 0.079
        p.pose.position.z = 0.0
        p.pose.orientation.x = -0.5
        p.pose.orientation.y = -0.5
        p.pose.orientation.z = 0.5
        p.pose.orientation.w = 0.5
        
        # 将tool附着到机器人的终端
        scene.attach_box(end_effector_link, 'tool', p, tool_size)
    #=======================================================
    #===================在场景中添加物体，防止碰撞====================    
        # 设置三维尺寸
        wall_size = [0.001, 2, 2]
        floor_size = [2, 2, 0.001]

        # 将wall加入场景当中
        wall_pose = PoseStamped()
        wall_pose.header.frame_id = 'wall'
        wall_pose.pose.position.x = 0.0
        wall_pose.pose.position.y = 0.0
        wall_pose.pose.position.z = 0.0
        wall_pose.pose.orientation.w = 1.0
        scene.add_box('wall', wall_pose, wall_size)
        
        rospy.sleep(2)  
    #=========================================================
        # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于base_link坐标系
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose = ccd_to_ur5_ik(-0.1,0,0,0)
        print target_pose.pose
        
        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()
        
        # 设置机械臂终端运动的目标位姿
        arm.set_pose_target(target_pose, end_effector_link)
        
        # 规划运动路径
        traj = arm.plan()
        # 按照规划的运动路径控制机械臂运动
        arm.execute(traj)
        rospy.sleep(1)

#=====================================================
        
#=====================================================



        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    MoveItIkDemo()

    
    

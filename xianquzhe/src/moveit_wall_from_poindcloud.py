#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
程序功能：
根据墙面tf、墙面方程控制ur5末端工作
'''


import rospy, sys
import moveit_commander
from moveit_commander import  MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose
from copy import deepcopy


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
                        
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'kinect_frame_optical'
        arm.set_pose_reference_frame(reference_frame)
                
        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
        arm.set_planning_time(10)
        
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.01)
       
        # 设置允许的最大速度和加速度
        arm.set_max_acceleration_scaling_factor(0.5)
        arm.set_max_velocity_scaling_factor(0.5)

        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()    

        # 控制机械臂先回到初始化位置
        arm.set_named_target('forward')
        arm.go()
        rospy.sleep(1)

        #===============加入墙面====================
        # 移除场景中之前运行残留的物体
        scene.remove_world_object('wall')

        # 设置wall的三维尺寸
        wall_size = [2, 2, 0.001]

        # 将wall加入场景当中
        wall_pose = PoseStamped()
        wall_pose.header.frame_id = 'wall'
        wall_pose.pose.position.x = 0
        wall_pose.pose.position.y = 0
        wall_pose.pose.position.z = 0
        wall_pose.pose.orientation.w = 1.0
        scene.add_box('wall', wall_pose, wall_size)
        
        rospy.sleep(2)  
        
        #=========移动到起点状态============= 
        # 更新当前的位姿
        arm.set_start_state_to_current_state()
        #从参数服务器读取墙面信息
        wall_tf = rospy.get_param("wall_tf")
        wall_coeff = rospy.get_param("wall_coeff")#墙面方程参数ABCD
      
        # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于base_link坐标系
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = -0.35
        target_pose.pose.position.y = 0.42
        #由平面方程求Z并-0.2
        target_pose.pose.position.z = -(wall_coeff[0]*target_pose.pose.position.x+wall_coeff[1]*target_pose.pose.position.y+wall_coeff[3])/wall_coeff[2]-0.2
        target_pose.pose.orientation.x = wall_tf[3]
        target_pose.pose.orientation.y = wall_tf[4] #-0.70729
        target_pose.pose.orientation.z = wall_tf[5]
        target_pose.pose.orientation.w = wall_tf[6] # 0.70729
        
        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()
        
        # 设置机械臂终端运动的目标位姿
        arm.set_pose_target(target_pose, end_effector_link)
        
        # 规划运动路径
        traj = arm.plan()
        
        # 按照规划的运动路径控制机械臂运动
        arm.execute(traj)
        rospy.sleep(1)

        #=================设置路径点====================================
        # 初始化路点列表
        waypoints = []
        wpose = deepcopy(target_pose.pose)

        # 将初始位姿加入路点列表
        waypoints.append(deepcopy(wpose))

        # 设置路点数据，并加入路点列表
        wpose.position.y -= 0.2
        wpose.position.z = -(wall_coeff[0]*wpose.position.x+wall_coeff[1]*wpose.position.y+wall_coeff[3])/wall_coeff[2]-0.2
        waypoints.append(deepcopy(wpose))

        wpose.position.x += 0.1
        wpose.position.z = -(wall_coeff[0]*wpose.position.x+wall_coeff[1]*wpose.position.y+wall_coeff[3])/wall_coeff[2]-0.2
        waypoints.append(deepcopy(wpose))

        wpose.position.y += 0.2
        wpose.position.z = -(wall_coeff[0]*wpose.position.x+wall_coeff[1]*wpose.position.y+wall_coeff[3])/wall_coeff[2]-0.2
        waypoints.append(deepcopy(wpose))

        wpose.position.x += 0.1
        wpose.position.z = -(wall_coeff[0]*wpose.position.x+wall_coeff[1]*wpose.position.y+wall_coeff[3])/wall_coeff[2]-0.2
        waypoints.append(deepcopy(wpose))

        wpose.position.y -= 0.2
        wpose.position.z = -(wall_coeff[0]*wpose.position.x+wall_coeff[1]*wpose.position.y+wall_coeff[3])/wall_coeff[2]-0.2
        waypoints.append(deepcopy(wpose))

        wpose.position.x += 0.1
        wpose.position.z = -(wall_coeff[0]*wpose.position.x+wall_coeff[1]*wpose.position.y+wall_coeff[3])/wall_coeff[2]-0.2
        waypoints.append(deepcopy(wpose))

        wpose.position.y += 0.2
        wpose.position.z = -(wall_coeff[0]*wpose.position.x+wall_coeff[1]*wpose.position.y+wall_coeff[3])/wall_coeff[2]-0.2
        waypoints.append(deepcopy(wpose))

        wpose.position.x += 0.1
        wpose.position.z = -(wall_coeff[0]*wpose.position.x+wall_coeff[1]*wpose.position.y+wall_coeff[3])/wall_coeff[2]-0.2
        waypoints.append(deepcopy(wpose))

        wpose.position.y -= 0.2
        wpose.position.z = -(wall_coeff[0]*wpose.position.x+wall_coeff[1]*wpose.position.y+wall_coeff[3])/wall_coeff[2]-0.2
        waypoints.append(deepcopy(wpose))

        wpose.position.x += 0.1
        wpose.position.z = -(wall_coeff[0]*wpose.position.x+wall_coeff[1]*wpose.position.y+wall_coeff[3])/wall_coeff[2]-0.2
        waypoints.append(deepcopy(wpose))

        wpose.position.y += 0.2
        wpose.position.z = -(wall_coeff[0]*wpose.position.x+wall_coeff[1]*wpose.position.y+wall_coeff[3])/wall_coeff[2]-0.2
        waypoints.append(deepcopy(wpose))

        wpose.position.x += 0.1
        wpose.position.z = -(wall_coeff[0]*wpose.position.x+wall_coeff[1]*wpose.position.y+wall_coeff[3])/wall_coeff[2]-0.2
        waypoints.append(deepcopy(wpose))

        wpose.position.y -= 0.2
        wpose.position.z = -(wall_coeff[0]*wpose.position.x+wall_coeff[1]*wpose.position.y+wall_coeff[3])/wall_coeff[2]-0.2
        waypoints.append(deepcopy(wpose))

        wpose.position.x += 0.1
        wpose.position.z = -(wall_coeff[0]*wpose.position.x+wall_coeff[1]*wpose.position.y+wall_coeff[3])/wall_coeff[2]-0.2
        waypoints.append(deepcopy(wpose))

        wpose.position.y += 0.2
        wpose.position.z = -(wall_coeff[0]*wpose.position.x+wall_coeff[1]*wpose.position.y+wall_coeff[3])/wall_coeff[2]-0.2
        waypoints.append(deepcopy(wpose))
        #=====================================================
        #笛卡尔路径规划
        fraction = 0.0   #路径规划覆盖率
        maxtries = 100   #最大尝试规划次数
        attempts = 0     #已经尝试规划次数
        
        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()

        # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
        while fraction < 1.0 and attempts < maxtries:
		    (plan, fraction) = arm.compute_cartesian_path (
		                            waypoints,   # waypoint poses，路点列表
		                            0.01,        # eef_step，终端步进值
		                            0.0,         # jump_threshold，跳跃阈值
		                            True)        # avoid_collisions，避障规划
		    
		    # 尝试次数累加
		    attempts += 1
		    
		    # 打印运动规划进程
		    if attempts % 10 == 0:
		        rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
		             
		# 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
        if fraction == 1.0:
		    rospy.loginfo("Path computed successfully. Moving the arm.")
		    arm.execute(plan)
		    rospy.loginfo("Path execution complete.")
		# 如果路径规划失败，则打印失败信息
        else:
		    rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  

        rospy.sleep(1)

        #=====================================================

        # 控制机械臂回到初始化位置
        arm.set_named_target('forward')
        arm.go()
        rospy.sleep(1)

        # 移除场景中之前运行残留的物体
        scene.remove_world_object('wall')
        rospy.sleep(1)

        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    MoveItIkDemo()

    
    

#!/usr/bin/env python
# coding=utf-8
from rosgraph_msgs.msg import Clock
import rospy
from gazebo_msgs.srv import GetModelStateRequest, GetModelState, GetWorldProperties
from geometry_msgs.msg import Twist
from move_base import *


if __name__ == '__main__':
    node = rospy.init_node('judement')
    rospy.wait_for_service('/gazebo/get_model_state')
    get_state_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)  # 服务代理

    model = GetModelStateRequest()  # 服务请求
    model.model_name = 'robot'
    objstate = get_state_service(model)  # 发送服务请求
    state = (objstate.pose.position.x, objstate.pose.position.y)
    print('--裁判启动（坐标偏离时开始计时）')
    last_x = state[0]
    start_flag = 0
    start_time = 0
    end_time = 0
    while not rospy.is_shutdown():
        rospy.sleep(1)
        objstate = get_state_service(model)  # 发送服务请求
        state = (objstate.pose.position.x, objstate.pose.position.y)
        if state[0] > 0.1  and start_flag == 0:
            print('--比赛开始！')
            start_flag = 1
            start_time = rospy.get_time()
        if  -0.269464 <= state[0] <= 0.017419 and -5.556189 <= state[1] <= -5.147794:
            print('--到达终点，比赛结束！')
            end_time = rospy.get_time()
            break
        # print(state)
    print('--用时'+str(end_time - start_time)+'秒(sim_time)')
    # rospy.spin()

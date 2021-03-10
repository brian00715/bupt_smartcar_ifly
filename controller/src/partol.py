#!/usr/bin/env python
 
import rospy
import actionlib
 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
 
 
waypoints = [  # <1>
# [(999,999, 0.0), (0.0, 0.0, 999, 999)],
   [(5.40672435388,-0.262692439825, 0.0), (0.0, 0.0, -0.667029915473, 0.745030933495)],
#    [(4.98868461081,-0.952945856667, 0.0), (0.0, 0.0, -0.93363183812, 0.358233988963)],
   [(2.53108708765,-0.990249105462, 0.0), (0.0, 0.0, -0.991439271644, 0.130568643408)],
   [(1.23305755052,-1.92948990839, 0.0), (0.0, 0.0, -0.360987894047, 0.932570501545)],
   [(1.25707282858,-3.04095572497, 0.0), (0.0, 0.0, -0.945214369985, 0.326450294492)],
#    [(1.18302500258,-4.17288084908, 0.0), (0.0, 0.0, 0.00547879224884, 0.999984991305)],
   [(3.518020401,-3.54373049857, 0.0), (0.0, 0.0, 0.295899426379, 0.955219100243)],
#    [(5.1868996315,-3.09252664751, 0.0), (0.0, 0.0, -0.106505869993, 0.994312073575)],
#    [(5.21603704132,-4.19247596592, 0.0), (0.0, 0.0, -0.830008578391, 0.557750625098)],
#    [(2.72480250898,-6.28348926909, 0.0), (0.0, 0.0, 0.995625495645, 0.0934337862974)],
   [(-0.157642994924,-5.24617583791, 0.0), (0.0, 0.0, 0.997500057221, 0.0706656624144)]
  

   
    
    
]
 
 
def goal_pose(pose):  # <2>
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]
 
    return goal_pose
 
 
if __name__ == '__main__':
    rospy.init_node('patrol')
 
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)  # <3>
    client.wait_for_server()
     
    while True:
        for pose in waypoints:   # <4>
            print("goal:x=%f y=%f"%(pose[0][0],pose[0][1]))
            goal = goal_pose(pose)
            client.send_goal(goal)
            client.wait_for_result()
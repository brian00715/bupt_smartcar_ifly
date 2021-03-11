## iFly-ROS常用指令

1. 加载世界

   ```sh
   roslaunch gazebo_pkg race.launch
   ```

2. 启动控制器

   键盘控制

   ```sh
   rosrun teleop_twist_keyboard teleop_twist_keyboard.py
   ```

   脚本控制

   ```sh
rosrun controller auto_navi.py
   ```
   
3. slam建图

   ```sh
   roslaunch ifly_slam ifly_slam.launch slam_methods:=karto
   ```

   保存地图

   ```sh
   rosrun map_server map_saver -f ~/ifly_ws/src/ifly_navigation/maps/map
   ```

4. 自动导航

   ```sh
   roslaunch ifly_navigation ifly_navigation.launch
   ```

   > 不出现costmap的解决方法：?? 每次运行前都跑一遍catkin_make
   
   自动更新中间点:
   
   ```sh
   rosrun ifly_navigation nav_keypoints.py
   ```
   
5. 比赛计时

   ```sh
   rosrun controller judement.py
   ```

6. 重置模型状态

   ```sh
   rosrun controller reset_model.py
   ```

7.打印当前坐标

   ```sh
   rostopic echo amcl_pose -n 1
   ```

   8.rqt_reconfigure

   ```sh
   rosrun rqt_reconfigure rqt_reconfigure
   ```
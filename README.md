# 第十六届智能车竞赛——讯飞智慧餐厅组别
Author：北京邮电大学SolveALL队 

Date：2021/03

本项目开源地址：https://gitee.com/SimonKenneth/bupt_smartcar_ifly **使用代码时请联系作者授权！**


## 介绍
+ 本仓库是对ros工作空间下src文件夹的镜像，使用时需先clone到本地，将默认的文件夹名改为`src`，然后copy到自己的工作空间下执行`catkin_make`。
+ 已经创建好了三个局部规划器的分支，调试时请切换到对应的分支。
+ 调试过程中请记录好参数与比赛时间的对应表格。
+ rqt_reconfigure报错的解决方案:
  在`/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/__init__.py`文件中找到`find_reconfigure_services`函数，将return处的语句改为：
    ```python
    return sorted([s[:-len('/set_parameters')] for s in rosservice.get_service_list() if s.endswith('/set_parameters') and s != '/set_parameters'])
    ```
## 目录说明
```c
|- controller // 虚拟遥控相关
|- gazebo_pkg // gazebo加载相关
|- ifly_navigation // ROS导航包
    |- param // 开源全局/局部规划期参数
    |- rviz // rviz配置文件，根据使用的规划期双击运行
    |- src
        |- pid.py // PID控制器实现
        |- nav_keypoints.py // 关键点导航脚本
        |- pure_pursuit.py // 纯追踪导航脚本
|- ifly_slam // slam建图包
|- start_game // 讯飞官方比赛启动器
```
         
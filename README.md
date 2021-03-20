# bupt_smartcar_ifly

#### 介绍
+ 本仓库是对ros工作空间下src文件夹的镜像，使用时需先clone到本地，将默认的文件夹名改为`src`，然后copy到自己的工作空间下执行`catkin_make`。
+ 已经创建好了三个局部规划器的分支，调试时请切换到对应的分支。
+ 调试过程中请记录好参数与比赛时间的对应表格。
+ rqt_reconfigure报错的解决方案:
> 在`/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/__init__.py`文件中找到`find_reconfigure_services`函数，将return处的语句改为：
```python
return sorted([s[:-len('/set_parameters')] for s in rosservice.get_service_list() if s.endswith('/set_parameters') and s != '/set_parameters'])
```
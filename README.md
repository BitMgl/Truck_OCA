# Truck_OCA

# FEISHU
https://fcnx730f4gdh.feishu.cn/docx/QoMVdFQrjoNzwAxMgXWcs0ABnrh?from=from_copylink

# Planner使用

emplanner功能包放置于工作空间/src文件夹下。

启动plan节点指令：

```
catkin build
source devel/setup.bash
rosrun em_planner_ros em_planner_ros_node  
```

测试用仿真环境

```
rosrun em_planner_ros environment_publisher_node

```

roslaunch启动方式

```
roslaunch show_planning_results_simple visualizer.launch 


```
# 0731-0801更新日志

1.规划起点用用实际定位，使用rqt发布/localization来实现

2.发布的参考轨迹/reference_line包含了固定的速度（20，在environmentpublisher.h文件中定义）。（但emplanner给出的qppath和dppath还是没有速度）

3.添加yaml支持，用于调参emplanner参数。（0801更新修改yaml的bug）

使用前安装yaml-cpp（正常来说是预装的）
```
sudo apt install libyaml-cpp-dev
```

# 0801_v2更新

修改emplanner.cpp sl_dp.cpp和h，构造qp问题的约束函数getcovexBound，将障碍物实际尺寸加入到求解代码中。

# 0802_v1更新

修改main2，输出speed规划的速度。规划速度在topic/qp_speed_result中

# 0803_v1更新

小车会动了。

![示意图](docx/20250803.gif)


# 0808_V3更新

添加了障碍物未来位置预测，使用障碍物yaw+v预测的。

修改了qp约束，硬约束正确了现在。（包含障碍物带yaw会旋转一下的情况，旋转非90*n度会使用外接矩形轮廓。

添加了yaml的一些新的。


（明天要解决：yaw传输需要接口。。）

速度规划还是不对（不会刹车以及不知道什么时候该加速什么时候该减速）。车辆宽度没写进yaml。障碍物尺寸没写进st_dp函数用于创造qp约束。之后可能修改障碍物代价描述，当障碍物太大的时候，对折比较好，避免那个sl_dp规划效果不对的情况。

# 0810_V2

1）修改dp_path/qp_path障碍物碰撞描述，利用frenet速度表示障碍物方向

2）增加画图命令，完成st图和速度规划发布

3）实现即停效果

4）修改st_qp障碍物tmin-tmax计算

## 待完成

1）障碍物尺寸到代价函数描述及效果测试

2）规则后处理模块

3）st相关yaml

# 0812 

1) 添加小车接口，实现rosbag play

2) 增加规划起点选择判定

# 0905 实车测试相关指令

启动底盘

```
cd UGV
source roscore_ins.sh

```
规划模块 （现阶段还需要修改bit_planner/config/road_ref中的参考路径）

```
cd WGV
catkin build 
chmod +x start_plan.sh
./start_plan.sh

```
录制 + 命名

```
rosbag record /chassis /odomData /obstacle_list /qp_planning_result /qp_speed_result

```
回放

```
rosbag play xxx --topics /odomData

```


1) roslaunch show---- visualize-----
2) rosbag play 6.bag

# Truck_OCA

# Enviroment_Publisher(暂时)

障碍物/自车自定义

```
<EnvironmentPublisher.cpp>
std::vector<ObstacleInfo> my_obs = 
// s - l - yaw in frenet coordination 
{{40.0, 0.0, -0.3},{50.0, 3.5, 0.0},{75.0, 0, 0.2},{95.0, 0, 0.0}};
EnvironmentPublisher env_pub(nh, my_obs, 10.0, 0.0);   // 主车从 (10 m, 0 m) 开始
```

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

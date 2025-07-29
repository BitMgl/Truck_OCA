#include "EnvironmentPublisher.h"
#include <bits/c++config.h>

#include <iostream>
#include <ros/ros.h>

using ObstacleInfo = std::tuple<double /*s*/, double /*l*/, double /*yaw_offset*/>;

int main(int argc, char** argv){
  ros::init(argc, argv, "environment_publisher");
  ros::NodeHandle nh;

  //EnvironmentPublisher env_pub(nh);
  std::vector<ObstacleInfo> my_obs = {
  // s - l - yaw in frenet coordination 
  {40.0, 0.0, -0.3},
  {50.0, 3.5, 0.0},
  {75.0, 0, 0.2},
  {95.0, 0, 0.0},
};
 
  EnvironmentPublisher env_pub(nh, my_obs, 10.0, 0.0);   // 主车从 (10 m, 0 m) 开始
  ros::Rate loop_rate(10);

  while (ros::ok()){
    env_pub.publishReferenceLine();
    env_pub.publishObstacleList();
    env_pub.publishLocalization();

    ros::spinOnce();
    loop_rate.sleep();
    ROS_INFO("ros::spinOnce!");
  }
  
  return 0;
}
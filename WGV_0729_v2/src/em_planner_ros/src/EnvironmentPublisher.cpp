#include "EnvironmentPublisher.h"
#include <bits/c++config.h>

#include <iostream>
#include <ros/ros.h>


int main(int argc, char** argv){
  ros::init(argc, argv, "environment_publisher");
  ros::NodeHandle nh;

  //EnvironmentPublisher env_pub(nh);
  std::vector<std::pair<double,double>> my_obs = {{30,0}, {40,-3.5}, {65,0},{75,3.5}};
  EnvironmentPublisher env_pub(nh,my_obs);
  ros::Rate loop_rate(1);

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
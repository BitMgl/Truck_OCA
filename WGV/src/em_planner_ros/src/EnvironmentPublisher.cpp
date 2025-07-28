#include "../include/sce3.h"

#include <bits/c++config.h>

#include <iostream>

#include <ros/ros.h>
#include "em_planner_ros/PlanningResult.h"
#include "em_planner_ros/ObstacleList.h"
#include "em_planner_ros/Obstacle.h"

class EnvironmentPublisher {
public:
  EnvironmentPublisher(ros::NodeHandle& nh) {
    reference_line_pub_ = nh.advertise<em_planner_ros::PlanningResult>("reference_line", 1, true);
    obstacle_list_pub_ = nh.advertise<em_planner_ros::ObstacleList>("obstacle_list", 1, true);

    ROS_INFO("Generating scenario...");
    scenario_.generateRoad(200.0);  // 生成 S 弯环境

    // 发布一次参考线与障碍物
    publishReferenceLine();
    publishObstacleList();
  }
private:
  void publishReferenceLine() {
    em_planner_ros::PlanningResult ref_msg;

    for (const auto& pt : scenario_.referenceLine_) {
      em_planner_ros::TrajectoryPoint msg;
      msg.index = pt.index;
      msg.x = pt.x;
      msg.y = pt.y;
      msg.angle = pt.angle;
      msg.xg = pt.xg;
      msg.yg = pt.yg;
      msg.zg = pt.zg;
      msg.global_angle = pt.global_angle;
      msg.curvature = pt.curvature;
      msg.d_curvature = pt.d_curvature;
      msg.direction = pt.direction;
      msg.length = pt.length;
      msg.v = pt.v;
      msg.a = pt.a;
      msg.t = pt.t;

      ref_msg.trajectory.push_back(msg);
    }

    reference_line_pub_.publish(ref_msg);
    ROS_INFO("Reference line published.");
  }

  void publishObstacleList() {
    em_planner_ros::ObstacleList list_msg;

    for (size_t i = 0; i < scenario_.obs_.obstacle_list.size(); ++i) {
      const auto& obs = scenario_.obs_.obstacle_list[i];
      em_planner_ros::Obstacle ob_msg;

      ob_msg.type = 1;  // 可调整
      ob_msg.id = static_cast<int>(i);

      ob_msg.traj_p.index = obs.traj_p.index;
      ob_msg.traj_p.x = obs.traj_p.x;
      ob_msg.traj_p.y = obs.traj_p.y;
      ob_msg.traj_p.angle = obs.traj_p.angle;
      ob_msg.traj_p.xg = obs.traj_p.xg;
      ob_msg.traj_p.yg = obs.traj_p.yg;
      ob_msg.traj_p.zg = obs.traj_p.zg;
      ob_msg.traj_p.global_angle = obs.traj_p.global_angle;
      ob_msg.traj_p.curvature = obs.traj_p.curvature;
      ob_msg.traj_p.d_curvature = obs.traj_p.d_curvature;
      ob_msg.traj_p.direction = obs.traj_p.direction;
      ob_msg.traj_p.length = obs.traj_p.length;
      ob_msg.traj_p.v = obs.traj_p.v;
      ob_msg.traj_p.a = obs.traj_p.a;
      ob_msg.traj_p.t = obs.traj_p.t;

      ob_msg.length = obs.length;
      ob_msg.width = obs.width;
      ob_msg.height = 1.5;      // 默认值
      ob_msg.confidence = 1.0;  // 默认值
      ob_msg.age = 0.0;         // 默认值

      list_msg.obstacle_list.push_back(ob_msg);
    }

    list_msg.obstacle_list_history.clear();  // 当前不发布历史

    obstacle_list_pub_.publish(list_msg);
    ROS_INFO("Obstacle list published.");
  }

  ros::Publisher reference_line_pub_;
  ros::Publisher obstacle_list_pub_;
  sce3 scenario_;
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "environment_publisher");
  ros::NodeHandle nh;

  EnvironmentPublisher env_pub(nh);

  ros::spinOnce();
  return 0;
}
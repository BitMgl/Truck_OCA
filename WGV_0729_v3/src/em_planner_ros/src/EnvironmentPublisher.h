#pragma once
#include "../include/sce4.h"
#include "ros/ros.h"

#include "em_planner_ros/PlanningResult.h"
#include "em_planner_ros/ObstacleList.h"
#include "em_planner_ros/Obstacle.h"
#include "em_planner_ros/Localization.h"
#include <tuple>

using ObstacleInfo = std::tuple<double /*s*/, double /*l*/, double /*yaw_offset*/>;

class EnvironmentPublisher
{
public:

  /* 1. 默认：3 个障碍物 + 主车起点 (5 m, 0 m) */
  EnvironmentPublisher(ros::NodeHandle& nh)
      : EnvironmentPublisher(nh,
                             {{50.0, 0.0, 0.0},
                              {110.0, -3.5, 0.0},
                              {68.0, 3.5, 0.0}},
                             5.0, 0.0) {}

  // 带参：外部任意 (s,l,yaw)
    // 2. 任意 (s,l,yaw) 列表
  EnvironmentPublisher(
      ros::NodeHandle& nh,
      const std::vector<ObstacleInfo>& obs_info,
      double ego_s, double ego_l)
      : scenario_(obs_info, ego_s, ego_l){
    reference_line_pub_  = nh.advertise<em_planner_ros::PlanningResult>("reference_line", 1, true);
    obstacle_list_pub_   = nh.advertise<em_planner_ros::ObstacleList>("obstacle_list", 1, true);
    localization_pub_    = nh.advertise<em_planner_ros::Localization>("localization", 1, true);
    ROS_INFO("Generating scenario...");
  }

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
      msg.frenet_info.s = pt.frenet_info.s;
      msg.frenet_info.s_d = pt.frenet_info.s_d;
      msg.frenet_info.s_dd = pt.frenet_info.s_dd;
      msg.frenet_info.s_ddd = pt.frenet_info.s_ddd;
      msg.frenet_info.l = pt.frenet_info.l ;
      msg.frenet_info.l_d = pt.frenet_info.l_d ;
      msg.frenet_info.l_dd = pt.frenet_info.l_dd ;
      msg.frenet_info.l_ddd = pt.frenet_info.l_ddd ;
      msg.frenet_info.l_ds = pt.frenet_info.l_ds ;
      msg.frenet_info.l_dds = pt.frenet_info.l_dds ;
      msg.frenet_info.l_ddds = pt.frenet_info.l_ddds ;
      msg.frenet_info.ds = pt.frenet_info.ds ;
    
      ref_msg.trajectory.push_back(msg);
    }

    reference_line_pub_.publish(ref_msg);
    ROS_INFO("Reference line published.");
  }

  void publishObstacleList() {
    em_planner_ros::ObstacleList list_msg;
    for (size_t i = 0; i < scenario_.obs_.obstacle_list.size(); ++i) {
      const auto& obs = scenario_.obs_.obstacle_list[i];
    // for (const auto& obs : scenario_.obs_){
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

void publishLocalization() {
  em_planner_ros::Localization loc_msg;

  // 这里把 sce3 里已经算好的 ego_loc_ 填进去
  loc_msg.xg = scenario_.ego_loc_.xg;
  loc_msg.yg = scenario_.ego_loc_.yg;
  loc_msg.zg = 0.0;               // 没有 z 信息时给 0
  loc_msg.yaw = scenario_.ego_loc_.yaw;
  loc_msg.pitch = 0.0;            // 没有俯仰角时给 0
  loc_msg.roll  = 0.0;            // 没有横滚角时给 0

  // 经纬度如果没有真值，可简单用 0 占位
  loc_msg.latitude  = 0.0;
  loc_msg.longitude = 0.0;
  loc_msg.altitude  = 0.0;

  localization_pub_.publish(loc_msg);
  ROS_INFO("Localization published.");
}

private:
  ros::Publisher reference_line_pub_;
  ros::Publisher obstacle_list_pub_;
  ros::Publisher localization_pub_;
  sce4 scenario_;
};

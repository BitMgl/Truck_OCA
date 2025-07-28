#include <ros/ros.h>
#include <iostream>
#include <map>
#include <string>

#include "matplotlibcpp.h"
#include "em_planner.h"
#include "sce1.h"
#include "sce2.h"
#include "sce3.h"

#include "em_planner_ros/PlanningResult.h"
#include "em_planner_ros/TrajectoryPoint.h"
#include "em_planner_ros/ObstacleList.h"
#include "em_planner_ros/Obstacle.h"


namespace plt = matplotlibcpp;

class PlannerNode {
public:
  PlannerNode() ; // 构造函数，里头初始化publisher
  void run();       // 原 main 函数的主要逻辑，后面我添加了publisher
  void referenceLineCallback(const em_planner_ros::PlanningResult::ConstPtr& msg);
  void obstacleCallback(const em_planner_ros::ObstacleList::ConstPtr& msg);
  std::vector<TrajectoryPoint> reference_line_;
  ObstacleList obs_;
  

private:
  ros::NodeHandle nh_;
  ros::Publisher result_pub_;
  ros::Subscriber refline_sub_;
  ros::Subscriber obstacles_sub_; //需要加什么新的subscriber直接在这个后面加。




  void rotate(std::vector<double>& ego_x, std::vector<double>& ego_y, double x,
              double y, double x_center, double y_center, double theta) {
    double rad = theta;  // 弧度
    double d = 2.236;    // 计算点之间的距离
    double alpha =
        std::atan2(y - y_center, x - x_center);  // 计算点 a 相对于点 b 的方位角
    alpha += rad;                                // 增加角度
    x = x_center + d * std::cos(alpha);  // 更新点 a 的 x 坐标
    y = y_center + d * std::sin(alpha);  // 更新点 a 的 y 坐标
    ego_x.push_back(x);
    ego_y.push_back(y);
  }

  void publishPlanningResult(const std::vector<TrajectoryPoint>& result_path) {
    em_planner_ros::PlanningResult msg;
    for (const auto& pt : result_path) {
      em_planner_ros::TrajectoryPoint pt_msg;
      pt_msg.index = pt.index;
      pt_msg.x = pt.x;
      pt_msg.y = pt.y;
      pt_msg.angle = pt.angle;
      pt_msg.xg = pt.xg;
      pt_msg.yg = pt.yg;
      pt_msg.zg = pt.zg;
      pt_msg.global_angle = pt.global_angle;
      pt_msg.curvature = pt.curvature;
      pt_msg.d_curvature = pt.d_curvature;
      pt_msg.direction = pt.direction;
      pt_msg.length = pt.length;
      pt_msg.v = pt.v;
      pt_msg.a = pt.a;
      pt_msg.t = pt.t;
      msg.trajectory.push_back(pt_msg);
      }
    result_pub_.publish(msg);
  }
};
  // 构造函数定义
  PlannerNode::PlannerNode() {
    result_pub_ = nh_.advertise<em_planner_ros::PlanningResult>("planning_result", 10);

    refline_sub_ = nh_.subscribe("reference_line", 1, &PlannerNode::referenceLineCallback, this);
    obstacles_sub_ = nh_.subscribe("obstacle_list", 1, &PlannerNode::obstacleCallback, this);
  }

  void PlannerNode::referenceLineCallback(const em_planner_ros::PlanningResult::ConstPtr& msg) {
    reference_line_.clear();
    for (const auto& pt_msg : msg->trajectory) {
      TrajectoryPoint pt;
      pt.index = pt_msg.index;
      pt.x = pt_msg.x;
      pt.y = pt_msg.y;
      pt.angle = pt_msg.angle;
      pt.xg = pt_msg.xg;
      pt.yg = pt_msg.yg;
      pt.zg = pt_msg.zg;
      pt.global_angle = pt_msg.global_angle;
      pt.curvature = pt_msg.curvature;
      pt.d_curvature = pt_msg.d_curvature;
      pt.direction = pt_msg.direction;
      pt.length = pt_msg.length;
      pt.v = pt_msg.v;
      pt.a = pt_msg.a;
      pt.t = pt_msg.t;
      reference_line_.push_back(pt);
    }
  }

  void PlannerNode::obstacleCallback(const em_planner_ros::ObstacleList::ConstPtr& msg) {
    obs_.obstacle_list.clear();

    for (const auto& obs_msg : msg->obstacle_list) {
      Obstacle obs;
      obs.id = obs_msg.id;
      obs.type = obs_msg.type;
      obs.width = obs_msg.width;
      obs.length = obs_msg.length;
      obs.height = obs_msg.height;
      obs.confidence = obs_msg.confidence;
      obs.age = obs_msg.age;

      // 拷贝 traj_p 字段
      obs.traj_p.index = obs_msg.traj_p.index;
      obs.traj_p.x = obs_msg.traj_p.x;
      obs.traj_p.y = obs_msg.traj_p.y;
      obs.traj_p.angle = obs_msg.traj_p.angle;
      obs.traj_p.xg = obs_msg.traj_p.xg;
      obs.traj_p.yg = obs_msg.traj_p.yg;
      obs.traj_p.zg = obs_msg.traj_p.zg;
      obs.traj_p.global_angle = obs_msg.traj_p.global_angle;
      obs.traj_p.curvature = obs_msg.traj_p.curvature;
      obs.traj_p.d_curvature = obs_msg.traj_p.d_curvature;
      obs.traj_p.direction = obs_msg.traj_p.direction;
      obs.traj_p.length = obs_msg.traj_p.length;
      obs.traj_p.v = obs_msg.traj_p.v;
      obs.traj_p.a = obs_msg.traj_p.a;
      obs.traj_p.t = obs_msg.traj_p.t;

      // 如不使用 obstacle_list_history，可忽略
      obs_.obstacle_list.push_back(obs);
    }
  }



  void PlannerNode::run() {
  // int main(int argc, char** argv) {
    sce3 sce_1;
    // step1--获取规划起点x,y,global_angle,v(vx,vy),a(ax,ay)
    TrajectoryPoint start_point;
    // start_point = FindFrenetProjPoint(sce_1.referenceLine_, 5);  //这个被他下四行替代掉。
    if (reference_line_.empty()) {
      ROS_WARN("Reference line not received yet.");
      return;
    }
    start_point = FindFrenetProjPoint(reference_line_, 5);

    // step2--根据规划起点更新参考线的s
    // updateRefLineS(start_point, sce_1.referenceLine_); //这个被他下1行替代掉。
    updateRefLineS(start_point, reference_line_);
    

    // step3--获取起点的frenet_info
    // CartesianToFrenet(start_point, sce_1.referenceLine_);
    CartesianToFrenet(start_point, reference_line_);

    // step3--更新障碍物的s,l
    for (auto& obs : obs_.obstacle_list) { //sce_1.
      CartesianToFrenet(obs.traj_p, reference_line_);  //sce_1.referenceLine_
    }

    EMPlanner em_planner(start_point, reference_line_,  //referenceLine_sce_1. 
                        obs_.obstacle_list);    //sce_1.

    em_planner.Plan();

    

    // Step 6: 获取结果并发布（假设 result_path 是 public 成员或 get 方法）
    const auto& result_path = em_planner.qp_path_;
    publishPlanningResult(result_path);
    
    // return 0;
  }


int main(int argc, char** argv) {
  ros::init(argc, argv, "planner_node");
  // result_pub_ = nh.advertise<em_planner_ros::PlanningResult>("planning_result", 10);

  PlannerNode node;
  ros::Rate rate(10);

  int wait_count = 0;
  while (ros::ok() && node.reference_line_.empty() &&node.obs_.obstacle_list.empty()) {
    ros::spinOnce();
    rate.sleep();
    wait_count++;
    if (wait_count > 100) {  // 最多等 10 秒
      ROS_ERROR("Timeout waiting for reference_line.");
      return 1;
    }
  }


  node.run();
  // publishPlanningResult(result_path);
  return 0;
}


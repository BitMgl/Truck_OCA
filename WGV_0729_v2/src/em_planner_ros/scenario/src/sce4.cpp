#include "../common/cartesian_frenet_conversion.h"
#include "../common/discrete_points_math.h"
#include "../include/sce4.h"
#include <cmath>

// 构造函数：接收 (s,l) 列表
sce4::sce4(const std::vector<std::pair<double, double>>& obs_sl_list)
    : obs_sl_list_(obs_sl_list.empty()
                       ? std::vector<std::pair<double, double>>{
                             {50.0, 0.0}, {110.0, -3.5}, {68.0, 3.5}}
                       : obs_sl_list) {
  generateRoad(200.0);
}

void sce4::generateRoad(double length) {
  const double radius   = 10.0;
  const double interval = 0.1;
  const int numPoints   = 315;
  const int numPoints1  = 843;
  const int numPoints2  = 842;
  const double deltaTheta = interval / radius;

  // 1. 生成中心线（S 弯）
  TrajectoryPoint temp;
  for (int i = 0; i < numPoints1; ++i) {
    temp.xg = i * interval - 84.3;
    temp.yg = radius;
    temp.length = i * 0.1;
    centerlane_.push_back(temp);
  }
  for (int i = 0; i < numPoints; ++i) {
    double theta = i * deltaTheta;
    temp.xg = -radius * std::cos(theta + 1.5708);
    temp.yg =  radius * std::sin(theta + 1.5708);
    temp.length = i * 0.1 + 0.1 * numPoints1;
    centerlane_.push_back(temp);
  }
  for (int i = 0; i < numPoints2; ++i) {
    temp.xg = -i * 0.1;
    temp.yg = -radius;
    temp.length = i * 0.1 + 0.1 * (numPoints + numPoints1);
    centerlane_.push_back(temp);
  }
  ComputePathProfile(centerlane_);

  // 2. 生成车道线/边界 Frenet 坐标
  size_t n = centerlane_.size();
  leftRoadBound_.resize(n);
  rightRoadBound_.resize(n);
  leftDashed_.resize(n);
  rightDashed_.resize(n);
  leftLane_.resize(n);
  rightLane_.resize(n);

  for (size_t i = 0; i < n; ++i) {
    double s = i * 0.1;

    auto setFrenet = [s](TrajectoryPoint& p, double l) {
      p.frenet_info.s = s;
      p.frenet_info.l = l;
    };

    setFrenet(leftRoadBound_[i],  5.25);
    setFrenet(rightRoadBound_[i], -5.25);
    setFrenet(leftDashed_[i],     1.75);
    setFrenet(rightDashed_[i],   -1.75);
    setFrenet(leftLane_[i],       3.5);
    setFrenet(rightLane_[i],     -3.5);
  }

  // Frenet -> Cartesian
  FrenetToCartesian(leftRoadBound_,  centerlane_);
  FrenetToCartesian(rightRoadBound_, centerlane_);
  FrenetToCartesian(leftDashed_,     centerlane_);
  FrenetToCartesian(rightDashed_,    centerlane_);
  FrenetToCartesian(leftLane_,       centerlane_);
  FrenetToCartesian(rightLane_,      centerlane_);

  // 3. 生成障碍物
  obs_.obstacle_list.clear();
  Obstacle obs;
  obs.length = 4.8;
  obs.width  = 2.0;
  obs.traj_p.v = 2.0;
  obs.traj_p.a = 0.0;

  for (const auto& sl : obs_sl_list_) {
    double s = sl.first;
    double l = sl.second;
    
    TrajectoryPoint proj = FindFrenetProjPoint(centerlane_, s);
    FrenetToCartesian(proj.length, proj.xg, proj.yg,
                      proj.global_angle, s, l,
                      obs.traj_p.xg, obs.traj_p.yg);
    
    // 如需角度微调，可在这里加：
    obs.traj_p.global_angle = proj.global_angle;   // 或根据需要减 0.3 等
    obs_.obstacle_list.push_back(obs);
  }

  // 4. 主车初始定位
  TrajectoryPoint ego_proj = FindFrenetProjPoint(centerlane_, 5.0);
  FrenetToCartesian(ego_proj.length, ego_proj.xg, ego_proj.yg,
                    ego_proj.global_angle, 5.0, 0.0,
                    ego_loc_.xg, ego_loc_.yg);
  ego_status_.v = 0.0;
  ego_status_.a = 0.0;

  // 5. 主车参考线
  referenceLine_ = centerlane_;
}
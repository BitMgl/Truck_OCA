#ifndef SCE4_H_
#define SCE4_H_
#include "../common/cartesian_frenet_conversion.h"
#include "../common/discrete_points_math.h"
#include "../common/data_struct.h"
#include <cmath>
#include <vector>
#include <tuple>   // std::pair

class sce4 {
  public:
    using ObstacleInfo = std::tuple<double, double, double>; // (s,l,yaw)

    /* 默认：3 个障碍物 + 主车起点(5,0) */
    sce4()
        : sce4({{50.0, 0.0, 0.0}, {110.0, -3.5, 0.0}, {68.0, 3.5, 0.0}}, 5.0, 0.0) {}

    /* 自定义：障碍物列表 + 主车起点(s,l) */
    explicit sce4(const std::vector<ObstacleInfo>& obs,
                  double ego_s, double ego_l)
        : obs_sl_list_(obs.empty()
                        ? std::vector<ObstacleInfo>{{50.0, 0.0, 0.0},
                                                    {110.0, -3.5, 0.0},
                                                    {68.0, 3.5, 0.0}}
                        : obs),
          ego_s_(ego_s),
          ego_l_(ego_l) {
        generateRoad(200.0);
    }

    ~sce4() = default;
  
    void generateRoad(double length) {
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

      for (const auto& info : obs_sl_list_) {
      double s   = std::get<0>(info);
      double l   = std::get<1>(info);
      double yaw_offset = std::get<2>(info);

      TrajectoryPoint proj = FindFrenetProjPoint(centerlane_, s);
      FrenetToCartesian(proj.length, proj.xg, proj.yg,
                        proj.global_angle, s, l,
                        obs.traj_p.xg, obs.traj_p.yg);

      obs.traj_p.global_angle = proj.global_angle + yaw_offset; // 完全自定义
      obs_.obstacle_list.push_back(obs);
      }

      // 4. 主车初始定位
      TrajectoryPoint ego_proj = FindFrenetProjPoint(centerlane_, ego_s_);
      FrenetToCartesian(ego_proj.length, ego_proj.xg, ego_proj.yg,
                        ego_proj.global_angle, ego_s_, ego_l_,
                        ego_loc_.xg, ego_loc_.yg);
      ego_status_.v = 0.0;   // 无速度
      ego_status_.a = 0.0;


      // 5. 主车参考线
      referenceLine_ = centerlane_;
    } 

 public:
  // 全局参考线及道路边界
  std::vector<TrajectoryPoint> referenceLine_;
  std::vector<TrajectoryPoint> leftRoadBound_;
  std::vector<TrajectoryPoint> rightRoadBound_;
  std::vector<TrajectoryPoint> leftDashed_;
  std::vector<TrajectoryPoint> rightDashed_;
  std::vector<TrajectoryPoint> centerlane_;
  std::vector<TrajectoryPoint> leftLane_;
  std::vector<TrajectoryPoint> rightLane_;

  ObstacleList obs_;          // 障碍物列表
  Localization ego_loc_;      // 主车定位
  VehicleStatus ego_status_;  // 主车状态



private:
  std::vector<ObstacleInfo> obs_sl_list_;
  double ego_s_, ego_l_;   // 主车初始 (s,l)
};

#endif  // sce4_H_
#ifndef SCE4_H_
#define SCE4_H_

#include "../common/data_struct.h"
#include <vector>
#include <utility>   // std::pair
class sce4 {
 public:
  // 默认 3 个障碍物
  sce4()
    : sce4({{50.0, 0.0}, {110.0, -3.5}, {68.0, 3.5}}) {}

   // 接收 (s,l) 列表
  explicit sce4(const std::vector<std::pair<double, double>>& obs_sl_list);

  ~sce4() = default;
  void generateRoad(double length);

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
  std::vector<std::pair<double, double>> obs_sl_list_;   // 存 (s,l
};

#endif  // sce4_H_
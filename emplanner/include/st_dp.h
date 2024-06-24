/***
 * @Author: dongtaihong 2396203400@qq.com
 * @Date: 2024-06-21 16:00:09
 * @LastEditTime: 2024-06-21 16:00:11
 * @FilePath: /AlgExper/emplanner/include/st_dp.h
 * @Description:
 */
#pragma once
#include <cmath>
#include <fstream>
#include <sstream>

#include "../../common/cartesian_frenet_conversion.h"
#include "../../common/eigen3/Eigen/Dense"
#include "iostream"

struct DpSpeedConfig {
  int rows = 0;
  int cols = 0;
  double ref_v = 0;

  double w_cost_obs = 0;
  double w_cost_ref_v = 0;
  double w_cost_a = 0;

  double dt = 0;
};

struct costStElement {
  double miniCost = 1e8;
  int preRow = -1;
  int preCol = -1;
  TrajectoryPoint pose;
};

struct obsST {  //匀速直线模型
  double tmin = 0.0;
  double tmax = 0.0;
  double sLeftMin = 0.0;
  double sLeftMax = 0.0;
  double sRightMin = 0.0;
  double sRightMax = 0.0;
};

class DpSpeed {
 public:
  /***
   * @description:
   * @param obstacle_array: 障碍物的st序列
   * @param start_point:
   * 起始点(轨迹拼接后的)的frenet坐标(包含各阶导)，s=0,s_d,s_dd
   * @param config: dp算法的配置
   * @param init_path: 输出的DpSpeed
   * @return {*}
   */
  void frenetDpSpeed(const TrajectoryPoint &start_point,
                     const DpSpeedConfig &config,
                     std::vector<TrajectoryPoint> &init_path);

  double calcNeighborCost(const TrajectoryPoint &start_point,
                          TrajectoryPoint &end_point,
                          const DpSpeedConfig &config);

  void trajectoryInterp(std::vector<TrajectoryPoint> &init_path,
                        const DpSpeedConfig &config);

  void getCovexBound(const std::vector<TrajectoryPoint> &init_path);

  void getObsStByCartesian(std::vector<Obstacle> &obs_list,
                           std::vector<TrajectoryPoint> &dp_path);

  std::vector<double> ub_, lb_;
  std::vector<obsST> obs_st_;
  double s_max_ = 0.0;
};
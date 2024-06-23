/***
 * @Author: dongtaihong 2396203400@qq.com
 * @Date: 2024-06-20 00:00:31
 * @LastEditTime: 2024-06-20 00:00:32
 * @FilePath: /AlgExper/emplanner/include/sl_dp.h
 * @Description:
 */
#pragma once
#include <cmath>
#include <fstream>
#include <sstream>

#include "../../common/data_struct.h"
#include "../../common/eigen3/Eigen/Dense"
#include "iostream"

struct DpPathConfig {
  int rows = 0;
  int cols = 0;
  double sample_s = 0;
  double sample_l = 0;

  double w_cost_obs = 0;
  double w_cost_dl = 0;
  double w_cost_ddl = 0;
  double w_cost_dddl = 0;
  double w_cost_ref = 0;

  double ds = 0;
};

struct costElement {
  double miniCost = 1e8;
  int preRow = -1;
  int preCol = -1;
  TrajectoryPoint pose;
};

class FrenetDpPath {
 public:
  /***
   * @description:
   * @param obstacle_array:
   * 障碍物的frenet序列（仅需要s,l,是在最新的frenet坐标系下-起点的s为0）
   * @param start_point: 起始点(轨迹拼接后的)的frenet坐标(包含各阶导)，s=0
   * @param config: dp算法的配置
   * @param init_path: 输出的frenet路径
   * @return {*}
   */
  void frenetDpPath(const std::vector<TrajectoryPoint> &obstacle_array,
                    const TrajectoryPoint &start_point,
                    const DpPathConfig &config,
                    std::vector<TrajectoryPoint> &init_path);
  double calcNeighborCost(const std::vector<TrajectoryPoint> &obstacle_array,
                          const TrajectoryPoint &start_point,
                          const TrajectoryPoint &end_point,
                          const DpPathConfig &config);
  void calcQuinticCoeff(const TrajectoryPoint &start_point,
                        const TrajectoryPoint &end_point,
                        Eigen::VectorXd &coeff);
  double calcObsCost(double w_cost_obs, double square_dist);
  void trajectoryInterp(std::vector<TrajectoryPoint> &init_path,
                        const DpPathConfig &config);
  void getCovexBound(const std::vector<TrajectoryPoint> &init_path,
                     const std::vector<TrajectoryPoint> &obstacle_array,
                     const DpPathConfig &config);

  std::vector<double> ub_, lb_;
};
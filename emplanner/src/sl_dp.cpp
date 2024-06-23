/***
 * @Author: dongtaihong 2396203400@qq.com
 * @Date: 2024-06-20 00:00:37
 * @LastEditTime: 2024-06-20 00:00:38
 * @FilePath: /AlgExper/emplanner/src/sl_dp.cpp
 * @Description:
 */
#include "../include/sl_dp.h"

#include <iomanip>

void FrenetDpPath::frenetDpPath(
    const std::vector<TrajectoryPoint> &obstacle_array,
    const TrajectoryPoint &start_point, const DpPathConfig &config,
    std::vector<TrajectoryPoint> &init_path) {
  init_path.clear();
  std::vector<std::vector<costElement> > cost_matrix;
  cost_matrix.resize(config.rows);
  for (int i = 0; i < config.rows; ++i) {
    cost_matrix[i].resize(config.cols);
  }
  for (int i = 0; i < config.rows; i++) {
    double l = (config.rows / 2 - i) * config.sample_l;
    for (int j = 0; j < config.cols; j++) {
      cost_matrix[i][j].pose.frenet_info.l = l;
      cost_matrix[i][j].pose.frenet_info.s =
          start_point.frenet_info.s + (j + 1) * config.sample_s;
      cost_matrix[i][j].pose.frenet_info.l_ds = 0;
      cost_matrix[i][j].pose.frenet_info.l_dds = 0;
    }
  }

  // calculate the first column cost
  for (int i = 0; i < config.rows; ++i) {
    double cost = calcNeighborCost(obstacle_array, start_point,
                                   cost_matrix[i][0].pose, config);
    cost_matrix[i][0].miniCost = cost;
  }

  for (int j = 1; j < config.cols; ++j) {
    for (int i = 0; i < config.rows; ++i) {
      for (int k = 0; k < config.rows; ++k) {
        double cost_temp =
            calcNeighborCost(obstacle_array, cost_matrix[k][j - 1].pose,
                             cost_matrix[i][j].pose, config);
        double pre_mini_cost = cost_matrix[k][j - 1].miniCost;
        double cost_cur = pre_mini_cost + cost_temp;
        if (cost_cur < cost_matrix[i][j].miniCost) {
          cost_matrix[i][j].miniCost = cost_cur;
          cost_matrix[i][j].preRow = k;
          cost_matrix[i][j].preCol = j - 1;
        }
      }
    }
  }

  int mini_cost_row = 0;
  int mini_cost_col = config.cols - 1;
  double mini_cost_last = 1e8;
  for (int i = 0; i < config.rows; ++i) {
    if (cost_matrix[i][mini_cost_col].miniCost < mini_cost_last) {
      mini_cost_last = cost_matrix[i][mini_cost_col].miniCost;
      mini_cost_row = i;
    }
  }

  init_path.push_back(cost_matrix[mini_cost_row][mini_cost_col].pose);
  for (int i = 0; i < config.cols - 1; ++i) {
    mini_cost_row = cost_matrix[mini_cost_row][mini_cost_col].preRow;
    mini_cost_col = cost_matrix[mini_cost_row][mini_cost_col].preCol;
    init_path.push_back(cost_matrix[mini_cost_row][mini_cost_col].pose);
  }
  init_path.push_back(start_point);
  std::reverse(init_path.begin(), init_path.end());
}

double FrenetDpPath::calcNeighborCost(
    const std::vector<TrajectoryPoint> &obstacle_array,
    const TrajectoryPoint &start_point, const TrajectoryPoint &end_point,
    const DpPathConfig &config) {
  Eigen::VectorXd coeff(6);
  calcQuinticCoeff(start_point, end_point, coeff);

  // use 10 points to calculate the cost
  double cost_sum = 0;
  for (int i = 0; i < 10; ++i) {
    double ds = start_point.frenet_info.s + i * config.sample_s / 10;
    double ds_2 = ds * ds, ds_3 = ds_2 * ds, ds_4 = ds_3 * ds, ds_5 = ds_4 * ds;
    double l = coeff[0] + coeff[1] * ds + coeff[2] * ds_2 + coeff[3] * ds_3 +
               coeff[4] * ds_4 + coeff[5] * ds_5;
    double l_ds = coeff[1] + 2 * coeff[2] * ds + 3 * coeff[3] * ds_2 +
                  4 * coeff[4] * ds_3 + 5 * coeff[5] * ds_4;
    double l_dds = 2 * coeff[2] + 6 * coeff[3] * ds + 12 * coeff[4] * ds_2 +
                   20 * coeff[5] * ds_3;
    double l_ddds = 6 * coeff[3] + 24 * coeff[4] * ds + 60 * coeff[5] * ds_2;

    double cost_smooth = config.w_cost_dl * pow(l_ds, 2) +
                         config.w_cost_ddl * pow(l_dds, 2) +
                         config.w_cost_dddl * pow(l_ddds, 2);
    double cost_ref = config.w_cost_ref * pow(l, 2);
    double cost_collision = 0;
    for (const auto &obstacle : obstacle_array) {
      Eigen::Vector2d pose_error = {obstacle.frenet_info.s - ds,
                                    obstacle.frenet_info.l - l};
      double square_dist = pose_error.dot(
          pose_error);  // Not Euclidean distance, just approximate
      double cost_collision_once = calcObsCost(config.w_cost_obs, square_dist);
      cost_collision += cost_collision_once;
    }

    cost_sum = cost_sum + cost_smooth + cost_ref + cost_collision;
  }
  return cost_sum;
}

void FrenetDpPath::calcQuinticCoeff(const TrajectoryPoint &start_point,
                                    const TrajectoryPoint &end_point,
                                    Eigen::VectorXd &coeff) {
  // l = a0 + a1*s +a2*s^2 + a3*s^3 + a4*s^4 + a5*s^5;
  // l' = a1 + 2*a2*s + 3*a3*s^2 + 4*a4*s^3 + 5*a5*s^4
  // l'' = 2*a2 + 6*a3*s + 12*a4*s^2 + 20*a5*s^3

  Eigen::Matrix<double, 6, 6> A;
  Eigen::VectorXd B(6);
  double S_2 = start_point.frenet_info.s * start_point.frenet_info.s;
  double S_3 = S_2 * start_point.frenet_info.s;
  double S_4 = S_3 * start_point.frenet_info.s;
  double S_5 = S_4 * start_point.frenet_info.s;

  double E_2 = end_point.frenet_info.s * end_point.frenet_info.s;
  double E_3 = E_2 * end_point.frenet_info.s;
  double E_4 = E_3 * end_point.frenet_info.s;
  double E_5 = E_4 * end_point.frenet_info.s;

  A << 1, start_point.frenet_info.s, S_2, S_3, S_4, S_5, 0, 1,
      2 * start_point.frenet_info.s, 3 * S_2, 4 * S_3, 5 * S_4, 0, 0, 2,
      6 * start_point.frenet_info.s, 12 * S_2, 20 * S_3, 1,
      end_point.frenet_info.s, E_2, E_3, E_4, E_5, 0, 1,
      2 * end_point.frenet_info.s, 3 * E_2, 4 * E_3, 5 * E_4, 0, 0, 2,
      6 * end_point.frenet_info.s, 12 * E_2, 20 * E_3;

  B << start_point.frenet_info.l, start_point.frenet_info.l_ds,
      start_point.frenet_info.l_dds, end_point.frenet_info.l,
      end_point.frenet_info.l_ds, end_point.frenet_info.l_dds;
  coeff = A.inverse() * B;
}

double FrenetDpPath::calcObsCost(double w_cost_obs, double square_dist) {
  // if dist>4 cost = 0; if dist in [3,4], cost = 1000/square_dist; if dist<3,
  // cost = w_cost_obs
  double cost = 0;
  if (square_dist >= 9 && square_dist <= 16)
    cost = 1000 / square_dist;
  else if (square_dist < 9)
    cost = w_cost_obs;
  else
    cost = 0;
  return cost;
}

void FrenetDpPath::trajectoryInterp(std::vector<TrajectoryPoint> &init_path,
                                    const DpPathConfig &config) {
  if (init_path.empty()) return;
  std::vector<TrajectoryPoint> ori_path = init_path;
  init_path.clear();
  double ds = config.ds;
  int point_size = config.cols * config.sample_s / ds + 1;
  init_path.reserve(point_size);

  init_path.push_back(ori_path.at(0));
  double s_cur = ori_path.at(0).frenet_info.s + ds;
  int count = 1;
  double safe_num = 1e-6;
  for (int i = 0; i < ori_path.size() - 1; ++i) {
    Eigen::VectorXd coeff(6);
    TrajectoryPoint p1 = ori_path.at(i), p2 = ori_path.at(i + 1);
    calcQuinticCoeff(p1, p2, coeff);
    //这里加safe_num是防止double的精度问题导致80.0 < 80.0的情况
    while ((s_cur + safe_num) < ori_path.at(i + 1).frenet_info.s) {
      TrajectoryPoint point_curr;
      point_curr.frenet_info.s = s_cur;
      double s_2 = s_cur * s_cur, s_3 = s_2 * s_cur, s_4 = s_3 * s_cur,
             s_5 = s_4 * s_cur;
      point_curr.frenet_info.l = coeff[0] + coeff[1] * s_cur + coeff[2] * s_2 +
                                 coeff[3] * s_3 + coeff[4] * s_4 +
                                 coeff[5] * s_5;
      point_curr.frenet_info.l_ds = coeff[1] + 2 * coeff[2] * s_cur +
                                    3 * coeff[3] * s_2 + 4 * coeff[4] * s_3 +
                                    5 * coeff[5] * s_4;
      point_curr.frenet_info.l_dds = 2 * coeff[2] + 6 * coeff[3] * s_cur +
                                     12 * coeff[4] * s_2 + 20 * coeff[5] * s_3;
      init_path.push_back(point_curr);
      s_cur += ds;
      count++;
      // if (count > point_size) return;
    }
  }
  init_path.push_back(ori_path.back());
}

int getMatchSIndex(const std::vector<TrajectoryPoint> &init_path, double s) {
  int count = 0, index = -1;
  if (s < init_path.at(0).frenet_info.s)
    return index = 0;
  else if (s > init_path.back().frenet_info.s)
    return index = init_path.size() - 1;
  else {
    for (auto &point : init_path) {
      if (point.frenet_info.s > s) break;
      count++;
    }
    if ((init_path.at(count).frenet_info.s - s) >
        (s - init_path.at(count - 1).frenet_info.s))
      return index = count;
    else
      return index = count - 1;
  }
}

void FrenetDpPath::getCovexBound(
    const std::vector<TrajectoryPoint> &init_path,
    const std::vector<TrajectoryPoint> &obstacle_array,
    const DpPathConfig &config) {
  if (init_path.empty()) return;
  lb_.resize(init_path.size(), -5.25);
  ub_.resize(init_path.size(), 5.25);  // left bound of vehicle
  for (auto &obs : obstacle_array) {
    double obs_s_min = obs.frenet_info.s - 4.8 / 2.0;
    double obs_s_max = obs.frenet_info.s + 4.8 / 2.0;
    int index_s_min = getMatchSIndex(init_path, obs_s_min);
    int index_s_max = getMatchSIndex(init_path, obs_s_max);
    int index_obs = getMatchSIndex(init_path, obs.frenet_info.s);
    if (index_s_min == 0 || index_s_max == 0) continue;
    if (init_path.at(index_obs).frenet_info.l >
        obs.frenet_info.l)  // dp-path on left of obstacles
    {
      for (int i = index_s_min; i <= index_s_max; ++i)
        lb_[i] = std::max(obs.frenet_info.l + (2.4 + 2.4) / 2.0, lb_[i]);
    } else  // dp-path on right of obstacles
    {
      for (int i = index_s_min; i <= index_s_max; ++i)
        ub_[i] = std::min(obs.frenet_info.l - (2.4 + 2.4) / 2.0, ub_[i]);
    }
  }
}
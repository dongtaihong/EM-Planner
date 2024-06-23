/***
 * @Author: dongtaihong 2396203400@qq.com
 * @Date: 2024-06-21 16:00:15
 * @LastEditTime: 2024-06-21 16:00:17
 * @FilePath: /AlgExper/emplanner/src/st_dp.cpp
 * @Description:
 */
#include "../include/st_dp.h"

void DpSpeed::frenetDpSpeed(const TrajectoryPoint &start_point,
                            const DpSpeedConfig &config,
                            std::vector<TrajectoryPoint> &init_path) {
  init_path.clear();
  std::vector<std::vector<costStElement> > cost_matrix;
  cost_matrix.resize(config.rows + 1);  //多一行放0
  for (int i = 0; i < cost_matrix.size(); ++i) {
    cost_matrix[i].resize(config.cols);
  }
  int low_rows = (int)(0.5 * config.rows);
  double low_res = s_max_ * 0.5 / low_rows;
  int high_rows = config.rows - low_rows;
  double high_res = s_max_ * 0.5 / high_rows;

  for (int j = 0; j < config.cols; j++) {
    cost_matrix[0][j].pose.frenet_info.s = 0;
    cost_matrix[0][j].pose.t = j + 1;
  }
  for (int i = 1; i <= low_rows; i++) {
    for (int j = 0; j < config.cols; j++) {
      cost_matrix[i][j].pose.frenet_info.s = i * low_res;
      cost_matrix[i][j].pose.t = j + 1;
    }
  }
  for (int i = 1; i <= high_rows; i++) {
    for (int j = 0; j < config.cols; j++) {
      cost_matrix[i + low_rows][j].pose.frenet_info.s =
          s_max_ * 0.5 + i * high_res;
      cost_matrix[i + low_rows][j].pose.t = j + 1;
    }
  }

  // calculate the first column cost
  for (int i = 0; i < cost_matrix.size(); ++i) {
    // std::cout << "cost_matrix[i][3]--s: "
    //           << cost_matrix[i][3].pose.frenet_info.s << std::endl;
    double cost = calcNeighborCost(start_point, cost_matrix[i][0].pose, config);
    cost_matrix[i][0].miniCost = cost;
    cost_matrix[i][0].pose.v = cost_matrix[i][0].pose.frenet_info.s / 1.0;
  }
  int j = 0;
  for (j = 1; j < config.cols; ++j) {
    for (int i = 0; i < cost_matrix.size(); ++i) {
      for (int k = 0; k < cost_matrix.size(); ++k) {
        double cost_temp = calcNeighborCost(cost_matrix[k][j - 1].pose,
                                            cost_matrix[i][j].pose, config);
        double pre_mini_cost = cost_matrix[k][j - 1].miniCost;
        double cost_cur = pre_mini_cost + cost_temp;
        // std::cout << "( " << k << " ,  " << j - 1 << " )----> "
        //           << "( " << i << " ,  " << j << " )" << std::endl;
        // std::cout << "miniCost: " << cost_cur
        //           << " pre_mini_cost: " << pre_mini_cost << std::endl;
        if (cost_cur < cost_matrix[i][j].miniCost) {
          cost_matrix[i][j].miniCost = cost_cur;
          cost_matrix[i][j].preRow = k;
          cost_matrix[i][j].preCol = j - 1;
          cost_matrix[i][j].pose.v =
              (cost_matrix[i][j].pose.frenet_info.s -
               cost_matrix[k][j - 1].pose.frenet_info.s) /
              1.0;
        }
      }
    }
    double mini_cost_col_cur_cost = 1e8;
    int mini_cost_row_cur = 0;
    //看当前列是否已经到达了s_max，到达了，则不用管后面的DP
    for (int m = 0; m < cost_matrix.size(); ++m) {
      if (cost_matrix[m][j].miniCost < mini_cost_col_cur_cost) {
        mini_cost_col_cur_cost = cost_matrix[m][j].miniCost;
        mini_cost_row_cur = m;
      }
    }
    if (cost_matrix[mini_cost_row_cur][j].pose.frenet_info.s > s_max_ - 5) {
      break;
    }
  }

  int mini_cost_row = 0;
  int mini_cost_col =
      (j == config.cols) ? config.cols - 1 : j;  // j是DP最优的列数
  double mini_cost_last = 1e8;
  for (int i = 0; i < cost_matrix.size(); ++i) {
    if (cost_matrix[i][mini_cost_col].miniCost < mini_cost_last) {
      mini_cost_last = cost_matrix[i][mini_cost_col].miniCost;
      mini_cost_row = i;
    }
  }

  init_path.push_back(cost_matrix[mini_cost_row][mini_cost_col].pose);
  int for_num = mini_cost_col - 1;
  std::cout << "mini_cost_col: " << mini_cost_col << std::endl;
  for (int i = 0; i <= for_num; ++i) {
    mini_cost_row = cost_matrix[mini_cost_row][mini_cost_col].preRow;
    mini_cost_col = cost_matrix[mini_cost_row][mini_cost_col].preCol;
    init_path.push_back(cost_matrix[mini_cost_row][mini_cost_col].pose);
    // std::cout << "row: " << mini_cost_row << " col: " << mini_cost_col
    //           << std::endl;
    std::cout << "preRow: " << cost_matrix[mini_cost_row][mini_cost_col].preRow
              << " preCol: " << cost_matrix[mini_cost_row][mini_cost_col].preCol
              << " t: " << cost_matrix[mini_cost_row][mini_cost_col].pose.t
              << " s: "
              << cost_matrix[mini_cost_row][mini_cost_col].pose.frenet_info.s
              << " v: " << cost_matrix[mini_cost_row][mini_cost_col].pose.v
              << " a: " << cost_matrix[mini_cost_row][mini_cost_col].pose.a
              << " cost: " << cost_matrix[mini_cost_row][mini_cost_col].miniCost
              << std::endl;
  }

  init_path.push_back(start_point);
  std::reverse(init_path.begin(), init_path.end());
}

double DpSpeed::calcNeighborCost(const TrajectoryPoint &start_point,
                                 TrajectoryPoint &end_point,
                                 const DpSpeedConfig &config) {
  // use 2 points to calculate the cost
  double cost_sum = 0;
  double ref_v_cost = 0.0;
  double acc_cost = 0.0;
  double obs_cost = 0.0;
  double speed = (end_point.frenet_info.s - start_point.frenet_info.s) / 1.0;
  ref_v_cost = config.w_cost_ref_v * pow(speed - config.ref_v, 2);
  double acc = (speed - start_point.v) / 1.0;
  if (acc > 4.1 || acc < -6) {
    acc_cost = 1e6;  // 限制加速度
  } else {
    acc_cost = config.w_cost_a * pow(acc, 2);
  }
  for (auto obs : obs_st_) {
    for (int i = 1; i <= 10; i++) {
      double time = start_point.t + i * 0.1;
      double cur_s =
          start_point.frenet_info.s +
          i * 0.1 * (end_point.frenet_info.s - start_point.frenet_info.s);
      if (obs.tmin <= time && obs.tmax >= time) {
        double portition = (time - obs.tmin) / (obs.tmax - obs.tmin);
        double s_min =
            obs.sLeftMin + portition * (obs.sRightMin - obs.sLeftMin);
        double s_max =
            obs.sLeftMax + portition * (obs.sRightMax - obs.sLeftMax);
        double distance_s_min = std::pow(cur_s - s_min, 2);
        double distance_s_max = std::pow(cur_s - s_max, 2);
        double distance_min = std::min(distance_s_min, distance_s_max);
        if (distance_min < 9) {
          obs_cost += 1e6;
        } else if (distance_min > 36) {
          obs_cost += 0;
        } else {
          obs_cost += config.w_cost_obs * (36 - distance_min);
          // std::cout << "cost_obs: " << obs_cost << std::endl;
        }
      }
    }
  }

  cost_sum = ref_v_cost + 1 * acc_cost + 1 * obs_cost;

  return cost_sum;
}

void DpSpeed::getObsStByCartesian(std::vector<Obstacle> &obs_list,
                                  std::vector<TrajectoryPoint> &dp_path) {
  obsST obs_st_tmp;
  for (auto &obs : obs_list) {
    // cardesian2Frenet(obs.traj_p, dp_path);
    CartesianToFrenet(obs.traj_p, dp_path);
    if (std::fabs(obs.traj_p.frenet_info.s) < 100 &&
        std::fabs(obs.traj_p.frenet_info.l) < 30) {
      if (obs.traj_p.frenet_info.l * obs.traj_p.frenet_info.l_d <
          -0.1) {  //逐渐靠近参考线
        // std::cout << "s: " << obs.traj_p.frenet_info.s << "\n\n"
        //           << "s_d: " << obs.traj_p.frenet_info.s_d << "\n"
        //           << "s_dd: " << obs.traj_p.frenet_info.s_dd << "\n"
        //           << "l: " << obs.traj_p.frenet_info.l << "\n"
        //           << "l_d: " << obs.traj_p.frenet_info.l_d << "\n"
        //           << "l_dd: " << obs.traj_p.frenet_info.l_dd << "\n"
        //           << "l_ds: " << obs.traj_p.frenet_info.l_ds << "\n"
        //           << "l_dds: " << obs.traj_p.frenet_info.l_dds << "\n\n";
        obs_st_tmp.tmin =
            std::fabs(obs.traj_p.frenet_info.l / obs.traj_p.frenet_info.l_d);
        obs_st_tmp.tmax =
            obs_st_tmp.tmin + std::fabs(2 / obs.traj_p.frenet_info.l_d);
        obs_st_tmp.sLeftMin = obs.traj_p.frenet_info.s +
                              obs.traj_p.frenet_info.s_d * obs_st_tmp.tmin -
                              2.4;
        obs_st_tmp.sLeftMax = obs.traj_p.frenet_info.s +
                              obs.traj_p.frenet_info.s_d * obs_st_tmp.tmin +
                              4.8;
        obs_st_tmp.sRightMin = obs.traj_p.frenet_info.s +
                               obs.traj_p.frenet_info.s_d * obs_st_tmp.tmax -
                               2.4;
        obs_st_tmp.sRightMax = obs.traj_p.frenet_info.s +
                               obs.traj_p.frenet_info.s_d * obs_st_tmp.tmax +
                               4.8;
        if (obs_st_tmp.tmin < 8 && obs_st_tmp.sLeftMin < s_max_) {
          obs_st_.push_back(obs_st_tmp);
          std::cout << "tmin: " << obs_st_tmp.tmin
                    << " tmax: " << obs_st_tmp.tmax
                    << " sLeftMin: " << obs_st_tmp.sLeftMin
                    << " sLeftMax: " << obs_st_tmp.sLeftMax
                    << " sRightMin: " << obs_st_tmp.sRightMin
                    << " sRightMax: " << obs_st_tmp.sRightMax << std::endl;
        }
      }
    }
  }
  std::cout << "attention obs number: " << obs_st_.size() << std::endl;
}

void DpSpeed::trajectoryInterp(std::vector<TrajectoryPoint> &init_path,
                               const DpSpeedConfig &config) {
  if (init_path.empty()) return;
  std::vector<TrajectoryPoint> ori_path;
  for (int i = 0; i < init_path.size() - 1; i++) {
    for (int j = 0; j < 10; j++) {
      TrajectoryPoint point_curr;
      point_curr.t = init_path.at(i).t + j * 0.1;
      point_curr.frenet_info.s = init_path.at(i).frenet_info.s +
                                 j * 0.1 *
                                     (init_path.at(i + 1).frenet_info.s -
                                      init_path.at(i).frenet_info.s);
      ori_path.push_back(point_curr);
    }
  }
  ori_path.push_back(init_path.back());
  init_path.clear();
  init_path = std::move(ori_path);
}

void DpSpeed::getCovexBound(const std::vector<TrajectoryPoint> &init_path) {
  int size = init_path.size();

  ub_.resize(size, s_max_);
  lb_.resize(size, 0.0);
  for (int i = 0; i < size; i++) {
    for (auto obs : obs_st_) {
      if (obs.tmin <= init_path[i].t && obs.tmax >= init_path[i].t) {
        double portition = (init_path[i].t - obs.tmin) / (obs.tmax - obs.tmin);
        double s_min =
            obs.sLeftMin + portition * (obs.sRightMin - obs.sLeftMin);
        double s_max =
            obs.sLeftMax + portition * (obs.sRightMax - obs.sLeftMax);
        if (init_path[i].frenet_info.s < s_min) {
          ub_[i] = std::min(ub_[i], s_min - 2);
        } else {
          lb_[i] = std::max(lb_[i], s_max + 2);
        }
      }
    }
  }
}
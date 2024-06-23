/***
 * @Author: dongtaihong 2396203400@qq.com
 * @Date: 2024-06-24 00:25:31
 * @LastEditTime: 2024-06-24 00:25:32
 * @FilePath: /AlgExper/emplanner/src/em_planner.cpp
 * @Description:
 */
#include "../include/em_planner.h"

void EMPlanner::mergeTrajectory(std::vector<TrajectoryPoint>& lat_path,
                                std::vector<TrajectoryPoint>& lon_path) {
  int last_index = 0;
  for (auto& point : lon_path) {
    if (last_index > 0) last_index -= 1;
    for (; last_index < lat_path.size() - 1; last_index++) {
      std::cout << "lon s: " << point.frenet_info.s
                << " lat s: " << lat_path[last_index].frenet_info.s
                << std::endl;
      if (point.frenet_info.s + 1e-5 >= lat_path[last_index].frenet_info.s &&
          point.frenet_info.s < lat_path[last_index + 1].frenet_info.s) {
        double proportion =
            (point.frenet_info.s - lat_path[last_index].frenet_info.s) /
            (lat_path[last_index + 1].frenet_info.s -
             lat_path[last_index].frenet_info.s);
        point.xg = lat_path[last_index].xg +
                   proportion *
                       (lat_path[last_index + 1].xg - lat_path[last_index].xg);
        point.yg = lat_path[last_index].yg +
                   proportion *
                       (lat_path[last_index + 1].yg - lat_path[last_index].yg);
        std::cout << "lat xg: " << lat_path[last_index].xg
                  << " lat yg: " << lat_path[last_index + 1].yg << std::endl;
        break;
      }
    }
  }
}

void EMPlanner::Plan() {
  // step1: sl dp
  DpPathConfig dp_config;
  dp_config.w_cost_obs = 1e6;
  dp_config.w_cost_dl = 300;
  dp_config.w_cost_ddl = 2000;
  dp_config.w_cost_dddl = 10000;
  dp_config.w_cost_ref = 20;
  dp_config.rows = 9;
  dp_config.cols = 10;
  dp_config.sample_s = 10;
  dp_config.sample_l = 1;
  dp_config.ds = 0.1;
  std::vector<TrajectoryPoint> obs_list;
  for (auto& obs : obs_) {
    obs_list.push_back(obs.traj_p);
  }
  dp_path_alg_.frenetDpPath(obs_list, start_point_, dp_config, dp_path_);
  dp_path_alg_.trajectoryInterp(dp_path_, dp_config);
  FrenetToCartesian(dp_path_, referenceLine_);
  dp_path_alg_.getCovexBound(dp_path_, obs_list, dp_config);

  // step2: sl qp
  PiecewiseJerkPathProblem sl_qp(
      dp_path_alg_.ub_, dp_path_alg_.lb_, dp_config.ds,
      {start_point_.frenet_info.l, start_point_.frenet_info.l_ds,
       start_point_.frenet_info.l_dds});
  qp_path_ = dp_path_;
  bool qp_success = sl_qp.Optimize(qp_path_);
  FrenetToCartesian(qp_path_, referenceLine_);
  bool flag_qp = ComputePathProfile(qp_path_);

  // step3: st dp
  DpSpeed dp_speed;
  DpSpeedConfig dp_speed_config;
  dp_speed_config.rows = 100;
  dp_speed_config.cols = 8;
  dp_speed_config.ref_v = 20;
  dp_speed_config.w_cost_obs = 10;
  dp_speed_config.w_cost_ref_v = 10;
  dp_speed_config.w_cost_a = 50;
  dp_speed_config.dt = 0.1;
  dp_speed_alg_.s_max_ = qp_path_.back().frenet_info.s;
  dp_speed_alg_.getObsStByCartesian(obs_, qp_path_);
  dp_speed_alg_.frenetDpSpeed(start_point_, dp_speed_config, dp_speed_);
  dp_speed_alg_.trajectoryInterp(dp_speed_, dp_speed_config);
  dp_speed_alg_.getCovexBound(dp_speed_);

  // step4: st qp
  PiecewiseJerkSpeedProblem st_qp(dp_speed_alg_.ub_, dp_speed_alg_.lb_, 0.1,
                                  {0, 0, 0});
  qp_speed_ = dp_speed_;
  st_qp.Optimize(qp_speed_);

  // step5: merge trajectory
  st_path_ = qp_speed_;
  mergeTrajectory(qp_path_, st_path_);
}
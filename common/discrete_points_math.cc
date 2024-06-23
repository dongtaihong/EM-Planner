/*
 * @Author: dongtaihong 2396203400@qq.com
 * @Date: 2024-06-17 20:17:36
 * @LastEditTime: 2024-06-17 20:40:37
 * @FilePath: /AlgExper/common/discrete_points_math.cc
 * @Description:
 */
#include "discrete_points_math.h"

#include <cmath>

bool ComputePathProfile(std::vector<TrajectoryPoint>& raw_cartesian_path) {
  if (raw_cartesian_path.size() < 2) {
    return false;
  }
  std::vector<double> dxs;
  std::vector<double> dys;
  std::vector<double> y_over_s_first_derivatives;
  std::vector<double> x_over_s_first_derivatives;
  std::vector<double> y_over_s_second_derivatives;
  std::vector<double> x_over_s_second_derivatives;

  // Get finite difference approximated dx and dy for heading and kappa
  // calculation
  std::size_t points_size = raw_cartesian_path.size();
  for (std::size_t i = 0; i < points_size; ++i) {
    double x_delta = 0.0;
    double y_delta = 0.0;
    if (i == 0) {
      x_delta = (raw_cartesian_path[i + 1].xg - raw_cartesian_path[i].xg);
      y_delta = (raw_cartesian_path[i + 1].yg - raw_cartesian_path[i].yg);
    } else if (i == points_size - 1) {
      x_delta = (raw_cartesian_path[i].xg - raw_cartesian_path[i - 1].xg);
      y_delta = (raw_cartesian_path[i].yg - raw_cartesian_path[i - 1].yg);
    } else {
      x_delta =
          0.5 * (raw_cartesian_path[i + 1].xg - raw_cartesian_path[i - 1].xg);
      y_delta =
          0.5 * (raw_cartesian_path[i + 1].yg - raw_cartesian_path[i - 1].yg);
    }
    dxs.push_back(x_delta);
    dys.push_back(y_delta);
  }

  // 得到每个点的航向
  for (std::size_t i = 0; i < points_size; ++i) {
    raw_cartesian_path[i].global_angle = std::atan2(dys[i], dxs[i]);
  }

  // 得到每个点的s
  double distance = 0.0;
  double fx = raw_cartesian_path[0].xg;
  double fy = raw_cartesian_path[0].yg;
  double nx = 0.0;
  double ny = 0.0;
  raw_cartesian_path[0].length = 0.0;
  raw_cartesian_path[0].frenet_info.s = 0.0;
  for (std::size_t i = 1; i < points_size; ++i) {
    nx = raw_cartesian_path[i].xg;
    ny = raw_cartesian_path[i].yg;
    double end_segment_s =
        std::sqrt((fx - nx) * (fx - nx) + (fy - ny) * (fy - ny));
    raw_cartesian_path[i].length = end_segment_s + distance;
    raw_cartesian_path[i].frenet_info.s = end_segment_s + distance;
    distance += end_segment_s;
    fx = nx;
    fy = ny;
  }

  // 获得x' y'
  for (std::size_t i = 0; i < points_size; ++i) {
    double xds = 0.0;
    double yds = 0.0;
    if (i == 0) {
      xds = (raw_cartesian_path[i + 1].xg - raw_cartesian_path[i].xg) /
            (raw_cartesian_path[i + 1].length - raw_cartesian_path[i].length);
      yds = (raw_cartesian_path[i + 1].yg - raw_cartesian_path[i].yg) /
            (raw_cartesian_path[i + 1].length - raw_cartesian_path[i].length);
    } else if (i == points_size - 1) {
      xds = (raw_cartesian_path[i].xg - raw_cartesian_path[i - 1].xg) /
            (raw_cartesian_path[i].length - raw_cartesian_path[i - 1].length);
      yds = (raw_cartesian_path[i].yg - raw_cartesian_path[i - 1].yg) /
            (raw_cartesian_path[i].length - raw_cartesian_path[i - 1].length);
    } else {
      xds =
          (raw_cartesian_path[i + 1].xg - raw_cartesian_path[i - 1].xg) /
          (raw_cartesian_path[i + 1].length - raw_cartesian_path[i - 1].length);
      yds =
          (raw_cartesian_path[i + 1].yg - raw_cartesian_path[i - 1].yg) /
          (raw_cartesian_path[i + 1].length - raw_cartesian_path[i - 1].length);
    }
    x_over_s_first_derivatives.push_back(xds);
    y_over_s_first_derivatives.push_back(yds);
  }

  // 获得x'' y''
  for (std::size_t i = 0; i < points_size; ++i) {
    double xdds = 0.0;
    double ydds = 0.0;
    if (i == 0) {
      xdds =
          (x_over_s_first_derivatives[i + 1] - x_over_s_first_derivatives[i]) /
          (raw_cartesian_path[i + 1].length - raw_cartesian_path[i].length);
      ydds =
          (y_over_s_first_derivatives[i + 1] - y_over_s_first_derivatives[i]) /
          (raw_cartesian_path[i + 1].length - raw_cartesian_path[i].length);
    } else if (i == points_size - 1) {
      xdds =
          (x_over_s_first_derivatives[i] - x_over_s_first_derivatives[i - 1]) /
          (raw_cartesian_path[i].length - raw_cartesian_path[i - 1].length);
      ydds =
          (y_over_s_first_derivatives[i] - y_over_s_first_derivatives[i - 1]) /
          (raw_cartesian_path[i].length - raw_cartesian_path[i - 1].length);
    } else {
      xdds =
          (x_over_s_first_derivatives[i + 1] -
           x_over_s_first_derivatives[i - 1]) /
          (raw_cartesian_path[i + 1].length - raw_cartesian_path[i - 1].length);
      ydds =
          (y_over_s_first_derivatives[i + 1] -
           y_over_s_first_derivatives[i - 1]) /
          (raw_cartesian_path[i + 1].length - raw_cartesian_path[i - 1].length);
    }
    x_over_s_second_derivatives.push_back(xdds);
    y_over_s_second_derivatives.push_back(ydds);
  }

  //计算曲率κ = |x″y′ - x′y″| / ((x′)^2 + (y′)^2)^(3/2)
  for (std::size_t i = 0; i < points_size; ++i) {
    double xds = x_over_s_first_derivatives[i];
    double yds = y_over_s_first_derivatives[i];
    double xdds = x_over_s_second_derivatives[i];
    double ydds = y_over_s_second_derivatives[i];
    double kappa =
        (xds * ydds - yds * xdds) /
        (std::sqrt(xds * xds + yds * yds) * (xds * xds + yds * yds) + 1e-6);
    raw_cartesian_path[i].curvature = kappa;
  }

  // 计算曲率的变化率
  for (std::size_t i = 0; i < points_size; ++i) {
    double dkappa = 0.0;
    if (i == 0) {
      dkappa =
          (raw_cartesian_path[i + 1].curvature -
           raw_cartesian_path[i].curvature) /
          (raw_cartesian_path[i + 1].length - raw_cartesian_path[i].length);
    } else if (i == points_size - 1) {
      dkappa =
          (raw_cartesian_path[i].curvature -
           raw_cartesian_path[i - 1].curvature) /
          (raw_cartesian_path[i].length - raw_cartesian_path[i - 1].length);
    } else {
      dkappa =
          (raw_cartesian_path[i + 1].curvature -
           raw_cartesian_path[i - 1].curvature) /
          (raw_cartesian_path[i + 1].length - raw_cartesian_path[i - 1].length);
    }
    raw_cartesian_path[i].d_curvature = dkappa;
  }
  return true;
}

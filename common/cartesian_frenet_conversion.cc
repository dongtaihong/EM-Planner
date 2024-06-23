/*
 * @Author: dongtaihong 2396203400@qq.com
 * @Date: 2024-06-16 23:12:51
 * @LastEditTime: 2024-06-24 00:17:18
 * @FilePath: /AlgExper/common/cartesian_frenet_conversion.cc
 * @Description:
 */
#include "cartesian_frenet_conversion.h"

#include <iostream>

// double NormalizeAngle(const double angle) {
//   double a = std::fmod(angle + M_PI, 2.0 * M_PI);
//   if (a < 0) {
//     a += M_PI;
//   } else {
//     a -= M_PI;
//   }
//   return a;
// }

// void getFrenetProjectPoint(
//     TrajectoryPoint frenet_point,
//     const std::vector<TrajectoryPoint>& ref_cartesian_path,
//     TrajectoryPoint& projectPoint) {
//   int match_index = -1;
//   for (int i = 0; i < ref_cartesian_path.size() - 1; ++i) {
//     if (ref_cartesian_path[i].frenet_info.s < frenet_point.frenet_info.s &&
//         ref_cartesian_path[i + 1].frenet_info.s > frenet_point.frenet_info.s)
//       match_index = i;
//   }
//   if (match_index == -1) return;
//   TrajectoryPoint match_point = ref_cartesian_path.at(match_index);
//   Eigen::Vector2d r_n = {match_point.x, match_point.y};
//   Eigen::Vector2d tao_n = {cos(match_point.global_angle),
//                            sin(match_point.global_angle)};
//   double ds = frenet_point.frenet_info.s - match_point.frenet_info.s;
//   Eigen::Vector2d r_r = r_n + ds * tao_n;
//   projectPoint.xg = r_r[0];
//   projectPoint.yg = r_r[1];
//   projectPoint.global_angle =
//       match_point.global_angle + ds * match_point.curvature;
//   projectPoint.curvature = (match_point.curvature +
//                             ref_cartesian_path.at(match_index + 1).curvature)
//                             /
//                            2;
// }

void FrenetToCartesian(std::vector<TrajectoryPoint>& frenet_path,
                       std::vector<TrajectoryPoint>& ref_cartesian_path) {
  // TrajectoryPoint project_point;
  // for (auto& point : frenet_path) {
  //   getFrenetProjectPoint(point, ref_cartesian_path, project_point);
  //   Eigen::Vector2d r_r = {project_point.x, project_point.y};
  //   Eigen::Vector2d tau_r = {cos(project_point.global_angle),
  //                            sin(project_point.global_angle)};
  //   Eigen::Vector2d n_r = {-sin(project_point.global_angle),
  //                          cos(project_point.global_angle)};
  //   Eigen::Vector2d r_h = r_r + point.frenet_info.l * n_r;
  //   point.xg = r_h[0];
  //   point.yg = r_h[1];
  //   point.global_angle =
  //       project_point.global_angle +
  //       atan2(point.frenet_info.l_ds,
  //             (1 - point.frenet_info.l * project_point.curvature));
  //   point.global_angle = NormalizeAngle(point.global_angle);

  //   double d_theta = point.global_angle - project_point.global_angle;
  //   point.curvature =
  //       ((point.frenet_info.l_dds +
  //         project_point.curvature * point.frenet_info.l_ds * tan(d_theta)) *
  //            pow(cos(d_theta), 2) *
  //            (1 / (1 - project_point.curvature * point.frenet_info.l)) +
  //        project_point.curvature) *
  //       cos(d_theta) *
  //       (1 / (1 - project_point.curvature * point.frenet_info.l));
  // }

  TrajectoryPoint proj_site;
  for (unsigned int iter_frenet = 0; iter_frenet < frenet_path.size();
       ++iter_frenet) {
    //根据s匹配最近的参考点
    proj_site = FindFrenetProjPoint(ref_cartesian_path,
                                    frenet_path[iter_frenet].frenet_info.s);

    FrenetToCartesian(proj_site.length, proj_site.xg, proj_site.yg,
                      proj_site.global_angle,
                      frenet_path[iter_frenet].frenet_info.s,
                      frenet_path[iter_frenet].frenet_info.l,
                      frenet_path[iter_frenet].xg, frenet_path[iter_frenet].yg);
  }
  std::cout << "proj_site xg: " << proj_site.xg << " yg: " << proj_site.yg
            << " s: " << frenet_path.back().frenet_info.s << " s_ref: "
            << ref_cartesian_path[ref_cartesian_path.size() - 2].frenet_info.s
            << " s_ref2: " << ref_cartesian_path.back().frenet_info.s
            << std::endl;
}

//不完整，仅sl，配合ComputePathProfile可以得到完整的笛卡尔坐标
void FrenetToCartesian(const double& rs, const double& rx, const double& ry,
                       const double& rtheta, const double& s_condition,
                       const double& d_condition, double& x, double& y) {
  const double cos_theta_r = std::cos(rtheta);
  const double sin_theta_r = std::sin(rtheta);

  x = rx - sin_theta_r * d_condition;
  y = ry + cos_theta_r * d_condition;
}

TrajectoryPoint FindFrenetProjPoint(
    const std::vector<TrajectoryPoint>& ref_cartesian_path, const double& s) {
  double min_dis = 9999.0;
  TrajectoryPoint proj_site;
  for (unsigned int index = 0; index < ref_cartesian_path.size() - 1; ++index) {
    if (s > ref_cartesian_path.back().frenet_info.s) {
      proj_site = std::move(ref_cartesian_path.back());
    }
    if (s >= ref_cartesian_path[index].frenet_info.s &&
        s < ref_cartesian_path[index + 1].frenet_info.s) {
      //线性插值得到投影点的site
      proj_site.frenet_info.s = s;
      double proportion = (s - ref_cartesian_path[index].frenet_info.s) /
                          (ref_cartesian_path[index + 1].frenet_info.s -
                           ref_cartesian_path[index].frenet_info.s);
      proj_site.xg = (1 - proportion) * ref_cartesian_path[index].xg +
                     proportion * ref_cartesian_path[index + 1].xg;
      proj_site.yg = (1 - proportion) * ref_cartesian_path[index].yg +
                     proportion * ref_cartesian_path[index + 1].yg;
      if (std::fabs(ref_cartesian_path[index].global_angle -
                    ref_cartesian_path[index + 1].global_angle) >= 6.28) {
        proj_site.global_angle = ref_cartesian_path[index + 1].global_angle;
      } else {
        proj_site.global_angle =
            (1 - proportion) * ref_cartesian_path[index].global_angle +
            proportion * ref_cartesian_path[index + 1].global_angle;
      }
      break;
    }
  }
  return proj_site;
}

void CartesianToFrenet(std::vector<TrajectoryPoint>& cartesian_path,
                       std::vector<TrajectoryPoint>& ref_cartesian_path) {
  for (auto& point : cartesian_path) {
    CartesianToFrenet(point, ref_cartesian_path);
  }
}

void CartesianToFrenet(TrajectoryPoint& cardesian_point,
                       std::vector<TrajectoryPoint>& ref_cartesian_path) {
  int match_index = getMatchPoint(cardesian_point, ref_cartesian_path);
  TrajectoryPoint proj_point;
  getProjectPoint(cardesian_point, ref_cartesian_path[match_index], proj_point);
  CartesianToFrenet(cardesian_point, proj_point);
}

void CartesianToFrenet(TrajectoryPoint& cardesian_point,
                       const TrajectoryPoint& project_point) {
  Eigen::Vector2d r_h = {cardesian_point.xg, cardesian_point.yg};
  Eigen::Vector2d r_r = {project_point.xg, project_point.yg};
  Eigen::Vector2d tao_r = {cos(project_point.global_angle),
                           sin(project_point.global_angle)};
  Eigen::Vector2d n_r = {-sin(project_point.global_angle),
                         cos(project_point.global_angle)};
  Eigen::Vector2d n_h = {-sin(cardesian_point.global_angle),
                         cos(cardesian_point.global_angle)};

  cardesian_point.frenet_info.s = project_point.frenet_info.s;
  cardesian_point.frenet_info.l = (r_h - r_r).dot(n_r);

  // TODO: 这里的vx,vy是全局坐标系下的，还是车辆坐标系下的？
  Eigen::Vector2d v = {cardesian_point.v * cos(cardesian_point.global_angle),
                       cardesian_point.v * sin(cardesian_point.global_angle)};
  cardesian_point.frenet_info.l_d = v.dot(n_r);
  std::cout << "cardesian2Frenet: v" << cardesian_point.v << "----"
            << cardesian_point.frenet_info.l_d << std::endl;
  cardesian_point.frenet_info.s_d =
      (1 / (1 - cardesian_point.frenet_info.l * project_point.curvature)) *
      v.dot(tao_r);
  if (fabs(cardesian_point.frenet_info.s_d) < 1e-8) {
    cardesian_point.frenet_info.l_ds = 0;
  } else {
    cardesian_point.frenet_info.l_ds =
        cardesian_point.frenet_info.l_d / cardesian_point.frenet_info.s_d;
  }

  Eigen::Vector2d a = {cardesian_point.a * cos(cardesian_point.global_angle),
                       cardesian_point.a * sin(cardesian_point.global_angle)};

  cardesian_point.frenet_info.l_dd =
      a.dot(n_r) -
      project_point.curvature *
          (1 - project_point.curvature * cardesian_point.frenet_info.l) *
          pow(cardesian_point.frenet_info.s_d, 2);
  // because d_k/ds is small, use 0 replace
  cardesian_point.frenet_info.s_dd =
      (1 / (1 - cardesian_point.frenet_info.l * project_point.curvature)) *
      (a.dot(tao_r) + 2 * project_point.curvature *
                          cardesian_point.frenet_info.l_ds *
                          pow(cardesian_point.frenet_info.s_d, 2));
  if (fabs(cardesian_point.frenet_info.s_d) < 1e-8) {
    cardesian_point.frenet_info.l_dds = 0;
  } else {
    cardesian_point.frenet_info.l_dds =
        (1 / pow(cardesian_point.frenet_info.s_d, 2)) *
        (cardesian_point.frenet_info.l_dd -
         cardesian_point.frenet_info.l_ds * cardesian_point.frenet_info.s_dd);
  }
}

int getMatchPoint(TrajectoryPoint hostPoint,
                  const std::vector<TrajectoryPoint>& vecTraj) {
  std::vector<double> distVec;
  for (int i = 0; i < vecTraj.size() - 1; ++i) {
    double tempDist = pow(hostPoint.xg - vecTraj.at(i).xg, 2) +
                      pow(hostPoint.yg - vecTraj.at(i).yg, 2);
    distVec.push_back(tempDist);
  }
  auto itr = std::min_element(distVec.begin(), distVec.end());
  return static_cast<int>(std::distance(distVec.begin(), itr));
}

void getProjectPoint(TrajectoryPoint hostPoint, TrajectoryPoint& matchPoint,
                     TrajectoryPoint& projectPoint) {
  Eigen::Vector2d hostPos = {hostPoint.xg, hostPoint.yg},
                  matchPointPos = {matchPoint.xg, matchPoint.yg};
  Eigen::Vector2d d = hostPos - matchPointPos;
  Eigen::Vector2d tao = {cos(matchPoint.global_angle),
                         sin(matchPoint.global_angle)};
  Eigen::Vector2d projectPose = matchPointPos + tao * (d.transpose() * tao);
  projectPoint.curvature = matchPoint.curvature;
  projectPoint.global_angle =
      matchPoint.global_angle +
      matchPoint.curvature * static_cast<double>((d.transpose() * tao));
  projectPoint.xg = projectPose[0];
  projectPoint.yg = projectPose[1];

  // s有方向的概念
  Eigen::Vector2d dir = {projectPoint.xg - matchPoint.xg,
                         projectPoint.yg - matchPoint.yg};
  double s_temp = std::sqrt(pow(projectPoint.xg - matchPoint.xg, 2) +
                            pow(projectPoint.yg - matchPoint.yg, 2));
  projectPoint.frenet_info.s = dir.dot(tao) > 0
                                   ? matchPoint.frenet_info.s + s_temp
                                   : matchPoint.frenet_info.s - s_temp;
}

void updateRefLineS(TrajectoryPoint& start_point,
                    std::vector<TrajectoryPoint>& ref_path) {
  int match_index = getMatchPoint(start_point, ref_path);
  TrajectoryPoint proj_point;
  getProjectPoint(start_point, ref_path[match_index], proj_point);
  for (auto& p : ref_path) {
    p.frenet_info.s -= proj_point.frenet_info.s;
  }
}
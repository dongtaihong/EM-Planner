/***
 * @Author: dongtaihong 2396203400@qq.com
 * @Date: 2024-06-18 21:42:48
 * @LastEditTime: 2024-06-18 21:42:49
 * @FilePath: /AlgExper/src/sce2 copy.cpp
 * @Description:
 */
/***
 * @Author: dongtaihong 2396203400@qq.com
 * @Date: 2024-06-18 20:08:19
 * @LastEditTime: 2024-06-18 20:08:20
 * @FilePath: /AlgExper/src/sce1 copy.cpp
 * @Description:
 */
/***
 * @Author: dongtaihong 2396203400@qq.com
 * @Date: 2024-06-16 23:01:14
 * @LastEditTime: 2024-06-16 23:01:15
 * @FilePath: /AlgExper/src/sce1.cpp
 * @Description:
 */
#include "../include/sce3.h"

#include <bits/c++config.h>

#include <iostream>
void sce3::generateRoad(double length) {
  // 1.生成S弯参考线
  const double radius = 10.0;   // 圆的半径（单位：m）
  const double interval = 0.1;  // 采样间隔（单位：m）
  const int numPoints = 315;    // 计算大约需要的点数
  const int numPoints1 = 843;
  const int numPoints2 = 842;
  const double deltaTheta = interval / radius;
  double theta = 0.0;
  TrajectoryPoint temp;
  for (std::size_t i = 0; i < numPoints1; ++i) {
    temp.xg = i * interval - 84.3;
    temp.yg = radius;
    temp.length = i * 0.1;
    centerlane_.push_back(temp);
  }

  for (std::size_t i = 0; i < numPoints; ++i) {
    theta = i * deltaTheta;
    temp.xg = -radius * std::cos(theta + 1.5708);
    temp.yg = radius * std::sin(theta + 1.5708);
    temp.length = i * 0.1 + 0.1 * numPoints1;
    centerlane_.push_back(temp);
  }
  for (std::size_t i = 0; i < numPoints2; ++i) {
    temp.xg = i * -0.1;
    temp.yg = -radius;
    temp.length = i * 0.1 + 0.1 * (numPoints + numPoints1);
    centerlane_.push_back(temp);
  }

  bool flag = ComputePathProfile(centerlane_);

  // 2.获得各线的frenet坐标
  std::size_t size = centerlane_.size();
  leftRoadBound_.clear();
  rightRoadBound_.clear();
  leftDashed_.clear();
  rightDashed_.clear();
  leftLane_.clear();
  rightLane_.clear();
  for (std::size_t i = 0; i < size; ++i) {
    TrajectoryPoint temp;
    temp.frenet_info.s = i * 0.1;

    temp.frenet_info.l = 5.25;
    leftRoadBound_.push_back(temp);

    temp.frenet_info.l = -5.25;
    rightRoadBound_.push_back(temp);

    temp.frenet_info.l = 1.75;
    leftDashed_.push_back(temp);

    temp.frenet_info.l = -1.75;
    rightDashed_.push_back(temp);

    temp.frenet_info.l = 3.5;
    leftLane_.push_back(temp);

    temp.frenet_info.l = -3.5;
    rightLane_.push_back(temp);
  }

  // 3.根据frenet坐标得到笛卡尔坐标
  FrenetToCartesian(leftRoadBound_, centerlane_);
  FrenetToCartesian(rightRoadBound_, centerlane_);
  FrenetToCartesian(leftDashed_, centerlane_);
  FrenetToCartesian(rightDashed_, centerlane_);
  FrenetToCartesian(leftLane_, centerlane_);
  FrenetToCartesian(rightLane_, centerlane_);

  // 4.生成障碍物列表
  Obstacle obs;
  TrajectoryPoint proj_site;
  proj_site = FindFrenetProjPoint(centerlane_, 50);
  FrenetToCartesian(proj_site.length, proj_site.xg, proj_site.yg,
                    proj_site.global_angle, 50, 0, obs.traj_p.xg,
                    obs.traj_p.yg);
  obs.traj_p.v = 2;
  obs.traj_p.a = 0;
  obs.traj_p.global_angle = proj_site.global_angle - 0.3;
  obs.length = 4.8;
  obs.width = 2;
  obs_.obstacle_list.push_back(obs);
  proj_site = FindFrenetProjPoint(centerlane_, 110);
  FrenetToCartesian(proj_site.length, proj_site.xg, proj_site.yg,
                    proj_site.global_angle, 110, -3.5, obs.traj_p.xg,
                    obs.traj_p.yg);
  obs.traj_p.global_angle = proj_site.global_angle;
  obs_.obstacle_list.push_back(obs);
  proj_site = FindFrenetProjPoint(centerlane_, 68);
  FrenetToCartesian(proj_site.length, proj_site.xg, proj_site.yg,
                    proj_site.global_angle, 68, 3.5, obs.traj_p.xg,
                    obs.traj_p.yg);
  obs.traj_p.global_angle = proj_site.global_angle;
  obs_.obstacle_list.push_back(obs);

  // 5.生成主车定位
  proj_site = FindFrenetProjPoint(centerlane_, 5);
  FrenetToCartesian(proj_site.length, proj_site.xg, proj_site.yg,
                    proj_site.global_angle, 5, 0, ego_loc_.xg, ego_loc_.yg);
  ego_status_.v = 0;
  ego_status_.a = 0;

  // 6.获得主车参考线，并Profile
  referenceLine_ = centerlane_;
}
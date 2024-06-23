/***
 * @Author: dongtaihong 2396203400@qq.com
 * @Date: 2024-06-16 23:01:14
 * @LastEditTime: 2024-06-16 23:01:15
 * @FilePath: /AlgExper/src/sce1.cpp
 * @Description:
 */
#include "../include/sce1.h"

#include <bits/c++config.h>
void sce1::generateRoad(double length) {
  // 1.生成直线参考线
  for (double i = 0; i < length; i += 0.1) {
    TrajectoryPoint temp;
    temp.xg = i;
    temp.yg = 0;
    temp.length = i;
    temp.global_angle = 0;
    centerlane_.push_back(temp);
  }

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
  obs.traj_p.xg = 50;
  obs.traj_p.yg = 0.0;
  obs.traj_p.v = 0;
  obs.traj_p.a = 0;
  obs.traj_p.global_angle = 0;
  obs.length = 4.8;
  obs.width = 2;
  obs_.obstacle_list.push_back(obs);
  obs.traj_p.xg = 80;
  obs.traj_p.yg = -3.5;
  obs_.obstacle_list.push_back(obs);
  obs.traj_p.xg = 68;
  obs.traj_p.yg = 3.5;
  obs_.obstacle_list.push_back(obs);

  // 5.生成主车定位
  ego_loc_.xg = 5;
  ego_loc_.yg = 0;
  ego_loc_.yaw = 0;
  ego_status_.v = 0;
  ego_status_.a = 0;

  // 6.获得主车参考线，并Profile
  referenceLine_ = centerlane_;
  bool flag = ComputePathProfile(referenceLine_);
}
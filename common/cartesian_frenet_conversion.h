/***
 * @Author: dongtaihong 2396203400@qq.com
 * @Date: 2024-06-16 23:13:11
 * @LastEditTime: 2024-06-16 23:13:12
 * @FilePath: /AlgExper/common/cartesian_frenet_conversion.h
 * @Description:
 */
#pragma once
#include <cmath>

#include "data_struct.h"
#include "eigen3/Eigen/Dense"

// 1.1.将一条sl序列，按照参考线笛卡尔坐标，转换为全局坐标
void FrenetToCartesian(std::vector<TrajectoryPoint>& frenet_path,
                       std::vector<TrajectoryPoint>& ref_cartesian_path);

// 1.2.将一个点的sl坐标，根据其匹配的参考点笛卡尔坐标，转换为全局坐标
void FrenetToCartesian(const double& rs, const double& rx, const double& ry,
                       const double& rtheta, const double& s_condition,
                       const double& d_condition, double& x, double& y);

// 1.3.根据frenet坐标的s，找到参考线笛卡尔坐标点的投影点
TrajectoryPoint FindFrenetProjPoint(
    const std::vector<TrajectoryPoint>& ref_cartesian_path, const double& s);

// 2.1.将一条笛卡尔路径，按照参考线笛卡尔坐标，转换为frenet坐标
void CartesianToFrenet(std::vector<TrajectoryPoint>& cartesian_path,
                       std::vector<TrajectoryPoint>& ref_cartesian_path);

// 2.2.将一个笛卡尔点，按照参考线笛卡尔坐标，转换为frenet坐标
void CartesianToFrenet(TrajectoryPoint& cardesian_point,
                       std::vector<TrajectoryPoint>& ref_cartesian_path);

// 2.3.将一个笛卡尔点，按照投影点笛卡尔坐标，转换为frenet坐标
void CartesianToFrenet(TrajectoryPoint& cardesian_point,
                       const TrajectoryPoint& project_point);

// 2.4.根据笛卡尔坐标，匹配最近点
int getMatchPoint(TrajectoryPoint hostPoint,
                  const std::vector<TrajectoryPoint>& vecTraj);

// 2.5.根据当前点笛卡尔坐标，以及匹配点笛卡尔坐标，并投影
void getProjectPoint(TrajectoryPoint hostPoint, TrajectoryPoint& matchPoint,
                     TrajectoryPoint& projectPoint);

// 3.1.根据起点的笛卡尔坐标，更新参考线的s
void updateRefLineS(TrajectoryPoint& start_point,
                    std::vector<TrajectoryPoint>& ref_path);

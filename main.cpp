/***
 * @Author: dongtaihong 2396203400@qq.com
 * @Date: 2024-06-17 23:18:34
 * @LastEditTime: 2024-06-17 23:18:36
 * @FilePath: /AlgExper/src/main.cpp
 * @Description:
 */
#include <iostream>
#include <map>
#include <string>

#include "../common/matplotlibcpp.h"
#include "../emplanner/include/em_planner.h"
#include "scenario/include/sce1.h"
#include "scenario/include/sce2.h"
#include "scenario/include/sce3.h"

namespace plt = matplotlibcpp;

void rotate(std::vector<double>& ego_x, std::vector<double>& ego_y, double x,
            double y, double x_center, double y_center, double theta) {
  double rad = theta;  // 弧度
  double d = 2.236;    // 计算点之间的距离
  double alpha =
      std::atan2(y - y_center, x - x_center);  // 计算点 a 相对于点 b 的方位角
  alpha += rad;                                // 增加角度
  x = x_center + d * std::cos(alpha);  // 更新点 a 的 x 坐标
  y = y_center + d * std::sin(alpha);  // 更新点 a 的 y 坐标
  ego_x.push_back(x);
  ego_y.push_back(y);
}

int main(int argc, char** argv) {
  sce3 sce_1;
  // step1--获取规划起点x,y,global_angle,v(vx,vy),a(ax,ay)
  TrajectoryPoint start_point;
  start_point = FindFrenetProjPoint(sce_1.referenceLine_, 5);

  // step2--根据规划起点更新参考线的s
  updateRefLineS(start_point, sce_1.referenceLine_);

  // step3--获取起点的frenet_info
  CartesianToFrenet(start_point, sce_1.referenceLine_);

  // step3--更新障碍物的s,l
  for (auto& obs : sce_1.obs_.obstacle_list) {
    CartesianToFrenet(obs.traj_p, sce_1.referenceLine_);
  }

  EMPlanner em_planner(start_point, sce_1.referenceLine_,
                       sce_1.obs_.obstacle_list);

  em_planner.Plan();

  /**************path plot************* */
  std::vector<double> dp_planning_x, dp_planning_y;
  std::vector<double> qp_planning_x, qp_planning_y;

  for (auto p : em_planner.dp_path_) {
    dp_planning_x.push_back(p.xg);
    dp_planning_y.push_back(p.yg);
  }

  for (auto p : em_planner.qp_path_) {
    qp_planning_x.push_back(p.xg);
    qp_planning_y.push_back(p.yg);
  }
  /**************path plot************* */

  /**************speed plot************* */
  std::vector<double> dp_speed_x, dp_speed_y;
  std::vector<double> qp_speed_x, qp_speed_y;
  std::vector<double> em_x, em_y;
  std::vector<double> ub_speed, lb_speed;
  std::vector<double> obs_x, obs_y;

  for (auto& obs : em_planner.dp_speed_alg_.obs_st_) {
    obs_x.push_back(obs.tmin);
    obs_x.push_back(obs.tmin);
    obs_x.push_back(obs.tmax);
    obs_x.push_back(obs.tmax);

    obs_y.push_back(obs.sLeftMin);
    obs_y.push_back(obs.sLeftMax);
    obs_y.push_back(obs.sRightMax);
    obs_y.push_back(obs.sRightMin);
  }

  for (auto& st : em_planner.dp_speed_) {
    std::cout << "t: " << st.t << " s: " << st.frenet_info.s << std::endl;
    dp_speed_x.push_back(st.t);
    dp_speed_y.push_back(st.frenet_info.s);
  }

  for (auto& st : em_planner.qp_speed_) {
    std::cout << "t: " << st.t << " s: " << st.frenet_info.s << std::endl;
    qp_speed_x.push_back(st.t);
    qp_speed_y.push_back(st.frenet_info.s);
  }

  for (auto p : em_planner.dp_speed_alg_.ub_) {
    ub_speed.push_back(p);
  }

  for (auto p : em_planner.dp_speed_alg_.lb_) {
    lb_speed.push_back(p);
  }

  for (auto p : em_planner.st_path_) {
    em_x.push_back(p.xg);
    em_y.push_back(p.yg);
    std::cout << "xg: " << p.xg << " yg: " << p.yg << std::endl;
  }

  /**************speed plot************* */

  std::cout << "path: " << sce_1.centerlane_.size() << std::endl;
  std::vector<double> leftRoadBound_x, leftRoadBound_y;
  std::vector<double> rightRoadBound_x, rightRoadBound_y;
  std::vector<double> leftDashed_x, leftDashed_y;
  std::vector<double> rightDashed_x, rightDashed_y;
  std::vector<double> referenceLine_x, referenceLine_y;

  for (auto p : sce_1.leftRoadBound_) {
    leftRoadBound_x.push_back(p.xg);
    leftRoadBound_y.push_back(p.yg);
  }

  for (auto p : sce_1.rightRoadBound_) {
    rightRoadBound_x.push_back(p.xg);
    rightRoadBound_y.push_back(p.yg);
  }

  for (auto p : sce_1.leftDashed_) {
    leftDashed_x.push_back(p.xg);
    leftDashed_y.push_back(p.yg);
  }

  for (auto p : sce_1.rightDashed_) {
    rightDashed_x.push_back(p.xg);
    rightDashed_y.push_back(p.yg);
  }

  for (auto p : sce_1.referenceLine_) {
    referenceLine_x.push_back(p.xg);
    referenceLine_y.push_back(p.yg);
  }

  /*** S-L **/
  plt::figure_size(3000, 3000);
  plt::suptitle("path plan (s-l)");
  plt::plot(leftRoadBound_x, leftRoadBound_y, "k");
  plt::plot(rightRoadBound_x, rightRoadBound_y, "k");
  plt::plot(leftDashed_x, leftDashed_y, "k--");
  plt::plot(rightDashed_x, rightDashed_y, "k--");
  plt::plot(referenceLine_x, referenceLine_y, "r");
  plt::plot(dp_planning_x, dp_planning_y, "g--");
  plt::plot(qp_planning_x, qp_planning_y, "r--");
  // 画障碍物
  for (auto obs : em_planner.obs_) {
    std::vector<double> x;
    std::vector<double> y;
    rotate(x, y, obs.traj_p.xg - 0.5 * obs.length,
           obs.traj_p.yg - 0.5 * obs.width, obs.traj_p.xg, obs.traj_p.yg,
           obs.traj_p.global_angle);
    rotate(x, y, obs.traj_p.xg - 0.5 * obs.length,
           obs.traj_p.yg + 0.5 * obs.width, obs.traj_p.xg, obs.traj_p.yg,
           obs.traj_p.global_angle);
    rotate(x, y, obs.traj_p.xg + 0.5 * obs.length,
           obs.traj_p.yg + 0.5 * obs.width, obs.traj_p.xg, obs.traj_p.yg,
           obs.traj_p.global_angle);
    rotate(x, y, obs.traj_p.xg + 0.5 * obs.length,
           obs.traj_p.yg - 0.5 * obs.width, obs.traj_p.xg, obs.traj_p.yg,
           obs.traj_p.global_angle);
    std::map<std::string, std::string> keywords;
    keywords.insert(std::pair<std::string, std::string>("color", "black"));
    plt::fill(x, y, keywords);
  }
  // 画主车
  std::vector<double> x_ego;
  std::vector<double> y_ego;
  rotate(x_ego, y_ego, sce_1.ego_loc_.xg - 2, sce_1.ego_loc_.yg - 1,
         sce_1.ego_loc_.xg, sce_1.ego_loc_.yg, sce_1.ego_loc_.yaw);
  rotate(x_ego, y_ego, sce_1.ego_loc_.xg - 2, sce_1.ego_loc_.yg + 1,
         sce_1.ego_loc_.xg, sce_1.ego_loc_.yg, sce_1.ego_loc_.yaw);
  rotate(x_ego, y_ego, sce_1.ego_loc_.xg + 2, sce_1.ego_loc_.yg + 1,
         sce_1.ego_loc_.xg, sce_1.ego_loc_.yg, sce_1.ego_loc_.yaw);
  rotate(x_ego, y_ego, sce_1.ego_loc_.xg + 2, sce_1.ego_loc_.yg - 1,
         sce_1.ego_loc_.xg, sce_1.ego_loc_.yg, sce_1.ego_loc_.yaw);
  std::map<std::string, std::string> keywords;
  keywords.insert(std::pair<std::string, std::string>("color", "blue"));
  plt::fill(x_ego, y_ego, keywords);
  /*** S-L **/

  /*** S-T **/
  plt::figure_size(3000, 3000);
  plt::suptitle("speed plan (s-t)");
  plt::plot(dp_speed_x, dp_speed_y, "k");
  plt::plot(qp_speed_x, qp_speed_y, "g");
  plt::plot(dp_speed_x, ub_speed, "r--");
  plt::plot(dp_speed_x, lb_speed, "r--");
  std::map<std::string, std::string> keywords_st;
  keywords_st.insert(std::pair<std::string, std::string>("color", "black"));
  plt::fill(obs_x, obs_y, keywords_st);
  /*** S-T **/

  /*** MERGE **/
  plt::figure_size(3000, 3000);
  plt::suptitle("em planner");
  plt::plot(leftRoadBound_x, leftRoadBound_y, "k");
  plt::plot(rightRoadBound_x, rightRoadBound_y, "k");
  plt::plot(leftDashed_x, leftDashed_y, "k--");
  plt::plot(rightDashed_x, rightDashed_y, "k--");
  plt::plot(em_x, em_y, "b");
  // 画障碍物
  for (auto obs : em_planner.obs_) {
    std::vector<double> x;
    std::vector<double> y;
    rotate(x, y, obs.traj_p.xg - 0.5 * obs.length,
           obs.traj_p.yg - 0.5 * obs.width, obs.traj_p.xg, obs.traj_p.yg,
           obs.traj_p.global_angle);
    rotate(x, y, obs.traj_p.xg - 0.5 * obs.length,
           obs.traj_p.yg + 0.5 * obs.width, obs.traj_p.xg, obs.traj_p.yg,
           obs.traj_p.global_angle);
    rotate(x, y, obs.traj_p.xg + 0.5 * obs.length,
           obs.traj_p.yg + 0.5 * obs.width, obs.traj_p.xg, obs.traj_p.yg,
           obs.traj_p.global_angle);
    rotate(x, y, obs.traj_p.xg + 0.5 * obs.length,
           obs.traj_p.yg - 0.5 * obs.width, obs.traj_p.xg, obs.traj_p.yg,
           obs.traj_p.global_angle);
    std::map<std::string, std::string> keywords2;
    keywords2.insert(std::pair<std::string, std::string>("color", "black"));
    plt::fill(x, y, keywords2);
  }
  plt::fill(x_ego, y_ego, keywords);
  /*** MERGE **/

  plt::show();

  return 0;
}
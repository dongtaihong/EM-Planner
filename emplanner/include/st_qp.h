/***
 * @Author: dongtaihong 2396203400@qq.com
 * @Date: 2024-06-20 23:56:20
 * @LastEditTime: 2024-06-20 23:56:21
 * @FilePath: /AlgExper/emplanner/include/st_qp.h
 * @Description:
 */
#pragma once
#include <chrono>
#include <iostream>
#include <map>
#include <memory>
#include <utility>
#include <vector>

#include "../../common/data_struct.h"
#include "../../common/osqp/include/osqp.h"

class PiecewiseJerkSpeedProblem {
 public:
  PiecewiseJerkSpeedProblem() = default;
  PiecewiseJerkSpeedProblem(const std::vector<double> ub,
                            const std::vector<double> lb,
                            const double delta_s,  //构造函数初始化
                            const std::array<double, 3>& x_init);

  ~PiecewiseJerkSpeedProblem() = default;  //析构函数

  bool Optimize(std::vector<TrajectoryPoint>& qp_path);

  // 计算q矩阵
  void CalculateKernel(std::vector<c_float>* P_data,  //计算 Hessian 矩阵
                       std::vector<c_int>* P_indices,
                       std::vector<c_int>* P_indptr);
  // 计算p矩阵
  void CalculateOffset(std::vector<c_float>* q);  //计算g矩阵
  //计算A矩阵
  void CalculateAffineConstraint(std::vector<c_float>* A_data,
                                 std::vector<c_int>* A_indices,
                                 std::vector<c_int>* A_indptr,
                                 std::vector<c_float>* lower_bounds,
                                 std::vector<c_float>* upper_bounds);

  virtual OSQPSettings* SolverDefaultSettings();

  OSQPData* FormulateProblem();

  void FreeData(OSQPData* data);  //释放 OSQPData 结构体中动态分配的内存。

  template <typename T>
  T* CopyData(const std::vector<T>& vec) {
    T* data = new T[vec.size()];
    memcpy(data, vec.data(), sizeof(T) * vec.size());
    return data;
  }

  std::array<double, 3> scale_factor_ = {1.0, 1.0, 1.0};  //比例因子
  int max_iter_ = 4000;
  size_t num_of_knots_;  //待平滑的点数

  // output(l,l',l'')
  std::vector<double> x_;
  std::vector<double> dx_;
  std::vector<double> ddx_;

  //初始点状态
  std::array<double, 3> x_init_;

  //边界值
  std::vector<std::pair<double, double>> x_bounds_;
  std::vector<std::pair<double, double>> dx_bounds_;
  std::vector<std::pair<double, double>> ddx_bounds_;
  std::pair<double, double> dddx_bound_;

  //各阶导权重
  double weight_x_;
  double weight_dx_;
  double weight_ddx_;
  double weight_dddx_;

  double delta_s_;

  //参考线
  bool has_x_ref_ = false;
  std::vector<double> x_ref_;
  double weight_x_ref_ = 0.0;

  //参考速度项
  bool has_dx_ref_ = false;
  double weight_dx_ref_ = 0.0;
  double dx_ref_ = 0.0;

  //曲率惩罚项
  std::vector<double> penalty_dx_;

  //终点项
  bool has_end_state_ref_ = false;                            //有无终点项
  std::array<double, 3> weight_end_state_ = {0.0, 0.0, 0.0};  //终点权重
  std::array<double, 3> end_state_ref_;                       //终点
};

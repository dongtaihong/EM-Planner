/***
 * @Author: dongtaihong 2396203400@qq.com
 * @Date: 2024-06-24 00:25:24
 * @LastEditTime: 2024-06-24 00:25:24
 * @FilePath: /AlgExper/emplanner/include/em_planner.h
 * @Description:
 */
#include "../../common/discrete_points_math.h"
#include "sl_dp.h"
#include "sl_qp.h"
#include "st_dp.h"
#include "st_qp.h"

class EMPlanner {
 public:
  EMPlanner() = default;
  EMPlanner(TrajectoryPoint start_point,
            std::vector<TrajectoryPoint> referenceLine,
            std::vector<Obstacle> obs)
      : start_point_(start_point), referenceLine_(referenceLine), obs_(obs) {}
  ~EMPlanner() = default;

  void mergeTrajectory(std::vector<TrajectoryPoint>& lat_path,
                       std::vector<TrajectoryPoint>& lon_path);
  void Plan();

  TrajectoryPoint start_point_;
  std::vector<TrajectoryPoint> referenceLine_;
  std::vector<Obstacle> obs_;
  std::vector<TrajectoryPoint> dp_path_;
  std::vector<TrajectoryPoint> qp_path_;
  std::vector<TrajectoryPoint> dp_speed_;
  std::vector<TrajectoryPoint> qp_speed_;
  std::vector<TrajectoryPoint> st_path_;

  FrenetDpPath dp_path_alg_;
  DpSpeed dp_speed_alg_;
};
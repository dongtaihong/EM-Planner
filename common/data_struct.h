/***
 * @Author: dongtaihong 2396203400@qq.com
 * @Date: 2024-06-16 23:02:35
 * @LastEditTime: 2024-06-16 23:02:35
 * @FilePath: /AlgExper/common/data_struct.h
 * @Description:
 */
#pragma once
#include <vector>
struct Localization {
  double latitude;
  double longitude;
  double altitude;
  double xg;
  double yg;
  double zg;
  double yaw;
  double pitch;
  double roll;
};

struct FrenetPoint {
  double s;
  double s_d;
  double s_dd;
  double s_ddd;
  double l;
  double l_d;
  double l_dd;
  double l_ddd;
  double l_ds;
  double l_dds;
  double l_ddds;
  double ds;
};

struct TrajectoryPoint {
  int index = 0;  //在全局路径中的index
  double x = 0;
  double y = 0;
  double angle = 0;
  double xg = 0;
  double yg = 0;
  double zg = 0;
  double global_angle = 0;
  double curvature = 0;    //曲率
  double d_curvature = 0;  //曲率变化率
  int direction = 0;       // 0-前进 1-倒车
  double length =
      0;  //与frenet的s不同，s是相对于车辆当前位置的s，length是全局路径中的s
  double v = 0;
  double a = 0;
  double t = 0;

  // frenet信息(如果不用则不管)
  FrenetPoint frenet_info;
};

struct Trajectory {
  std::vector<TrajectoryPoint> trajectory;       //局部规划的轨迹
  std::vector<TrajectoryPoint> last_trajectory;  //上一帧局部规划的轨迹
  std::vector<TrajectoryPoint> current_global_path;  //当前位置的全局轨迹
};

struct Obstacle {
  int type;  // 0-unknown 1-small 2-mid 3-big
             // 4-pedestrian 5-bike 6-car 7-truck 8-motor
             // 9-other_vehicle 10-barrier 11-sign
  int id;
  TrajectoryPoint traj_p;
  double width;       // no feedback
  double length;      // no feedback
  double height;      // no feedback
  double confidence;  //置信度
  double age;         //生命长度
};

struct ObstacleList {
  std::vector<Obstacle> obstacle_list;
  std::vector<Obstacle> obstacle_list_history;  //需要历史信息做轨迹预测
};
struct VehicleStatus {
  //速度
  double vx;
  double vy;
  double v;
  //横摆角速度
  double yaw_rate;  //仿真中可以拿到，但实车上很难获取
  //加速度
  double ax;
  double ay;
  double a;
  //前轮转角
  double steer_front_wheel;  //仿真中可以拿到，但实车上很难获取
  //档位
  double shift_position;
  //油门开度、刹车力度
  double throttle;
  double brake;
};

struct VehicleParam {
  double length;
  double width;
  double wheel_base;                 //轴距
  double min_turn_radius;            //最小转弯半径
  double cornering_stiffness_front;  //前轮侧偏刚度
  double cornering_stiffness_rear;   //后轮侧偏刚度
  double mass_fl;                    //左前轮载荷
  double mass_fr;                    //右前轮载荷
  double mass_rl;                    //左后轮载荷
  double mass_rr;                    //右后轮载荷
  double mass;                       //总质量
  double a;                          //前轴中心到质心的距离
  double b;                          //后轴中心到质心的距离
};
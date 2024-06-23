<!--
 * @Author: dongtaihong 2396203400@qq.com
 * @Date: 2024-06-16 22:44:45
 * @LastEditTime: 2024-06-18 00:02:40
 * @FilePath: /AlgExper/README.md
 * @Description: 
-->
1.复制agileButterflyPnc的数据结构；done
2.笛卡尔坐标系与frenet坐标系的互相转换，done
  笛卡尔坐标系与车辆坐标系的互相转换；wait
3.构造三个场景的中心线，两点之间间隔0.1m；done
4.全局路径点的profile(根据x,y补全其他的信息global_angle， curvature，d_curvature)，获得完善的参考线信息;done
5.平移中心线，得到各场景的道路信息；done
6.障碍物的全局坐标、尺寸、航向、速度；done
7.matplotlib-cpp绘制图片；done



8.em planner的输入：参考线信息、道路边界信息、车辆定位信息、障碍物信息，输出：笛卡尔坐标系轨迹点；
  lattice planner的输入：参考线信息、道路边界信息、车辆定位信息、障碍物信息，输出：笛卡尔坐标系轨迹点；
  mpc planner的输入：参考线信息、道路边界信息、车辆定位信息、障碍物信息，输出：笛卡尔坐标系轨迹点；
  所以统一算法的输入输出接口如上。

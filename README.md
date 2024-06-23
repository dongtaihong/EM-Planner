##说明
不需要配置第三方库，所有第三方库已经放到代码common文件中（eigen，osqp，matplotlibcpp）

##使用
git clone https://github.com/dongtaihong/emplanner.git
cd emplanner
mkdir build && cd build
cmake ..
make
./planning_experiment

##TODO
5帧连续切片场景的规划仿真：以验证emplanner在动态Planning时，轨迹的抖动是否严重。

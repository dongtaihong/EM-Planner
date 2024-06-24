## 说明
不需要配置第三方库，所有第三方库已经放到代码common文件夹中（eigen，osqp，matplotlibcpp），如果编译出现python相关的错误，需要将CMakeLists.txt中的python版本修改为电脑默认版本

## 编译
```shell
git clone https://github.com/dongtaihong/emplanner.git
cd emplanner
mkdir build && cd build
cmake ..
make
./planning_experiment
```

## 仿真效果

#### path plan

![sl](./result/sl.png)

#### speed plan

![st](./result/st.png)

#### final trajectory

![emplanner](./result/emplanner.png)

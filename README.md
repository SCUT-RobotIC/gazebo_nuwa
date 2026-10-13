# gazebo仿真
## 说明
## 目录
## 1.地图
地图相关文件存在field文件夹中：  
```
field/
├── field_config.py        ← 所有尺寸 / 颜色 / 质量都在这，改这里
├── gen_world.py           ← 生成 SDF world
├── render_plan.py         ← 生成俯视平面图（开 Gazebo 之前先对图用）
├── worlds/
│   └── nuwa_field.sdf     ← 地图文件
└── out/
    └── plan.png           ← 2D地图文件
```
### 1.1 启动与关闭地图
```bash
env -u QT_XCB_GL_INTEGRATION gz sim
  /home/wyq/gazebo_nuwa/field/worlds/nuwa_field.sdf
```
```bash
pkill -f "[n]uwa_field.sdf"
```
## 2.小车

### 2.1启动小车模型
解析urdf文件：  
```bash
source /opt/ros/humble/setup.bash
  ros2 run robot_state_publisher robot_state_publisher \
    --ros-args -p robot_description:="$(cat .../mini_car.urdf)"
```
把模型塞进gazebo
```bash
source /opt/ros/humble/setup.bash
  ros2 run ros_gz_sim create -topic robot_description -name mini_car -x 0 -y -5 -z 0.3
```
### 2.2xacro
记得关掉conda，不然会和ros2冲突
```bash
conda deactivate
cd /home/wyq/gazebo_nuwa/robots/src/mini_car
```
检查xacro是否正确
xacro urdf/mini_car.xacro | head -20  
xacro urdf/mini_car.xacro > /tmp/t.urdf && check_urdf /tmp/t.urdf

## 3.启动
### 3.1启动地图与简单小车
```bash
ros2 launch ~/gazebo_nuwa/robots/src/mini_car/launch/mini_car.launch.py
```
### 3.2启动控制器
```bash
ros2 run mini_car_control teleop_wasd
```
### 3.3启动rviz2,带配置文件
```bash
rviz2 -d ~/gazebo_nuwa/robots/src/mini_car/rviz/mini_car.rviz
```

## 测试2D雷达
```bash
ros2 launch ~/gazebo_nuwa/robots/src/mini_car/launch/mini_car.launch.py \
    spawn_x:=0.0 spawn_y:=-2.50 spawn_z:=0.9
```
②放方块(两块叠在左边,一块在右边 ✓)

cd ~/gazebo_nuwa/robots/src/mini_car

  BLK=/home/wyq/gazebo_nuwa/robots/src/mini_car/models/block.sdf
  ros2 run ros_gz_sim create -file $BLK -name blk_stack_1 -x -2.75 -y -2.75 -z 0.778
  ros2 run ros_gz_sim create -file $BLK -name blk_stack_2 -x -2.75 -y -2.75 -z 1.130
  ros2 run ros_gz_sim create -file $BLK -name blk_single  -x  2.75 -y -2.75 -z 0.778


  ③ 看扫描

  ros2 topic echo /scan --once > /tmp/scan.txt
  wc -l /tmp/scan.txt

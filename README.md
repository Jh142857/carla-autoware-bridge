# carla-autoware-bridge
转发carla真实定位信息的carla-autoware-bridge

参照1安装autoware galactic版本与carla 0.9.13

这里借助官方的carla-ros-bridge将carla中的真值信息发送至ros2中

terminal1-carla

```bash
cd ${CARLA_ROOT}/CARLA_0.9.13
./CarlaUE4.sh
```

terminal2-carla-ros-bridge

```bash
git clone git@github.com:carla-simulator/ros-bridge.git carla-ros-bridge
cd carla-ros-bridge
# optional: if built earlier, skip
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 20
source install/setup.bash
ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py
```

terminal3-autoware galactic

```bash
cd ${AUTOWARE_ROOT}
source install/setup.bash
ros2 launch autoware_launch autoware.launch.xml map_path:=$HOME/autoware_map/Town01 vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
```

terminal4-carla-autoware-bridge：运行自己编写的消息转发节点（放入autoware.universe external）

```bash
cd ${AUTOWARE_ROOT}/src/universe/external
git clone git@github.com:Jh142857/carla-autoware-bridge.git
cd ${AUTOWARE_ROOT}
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 20 --packages-select carla_autoware_bridge 
source install/setup.bash
ros2 launch carla_autoware_bridge carla_autoware_bridge.launch.xml
```

即可实现carla真实定位信息转发至autoware

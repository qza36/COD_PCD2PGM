# GridMap生成2.5D高程地图
将pcd点云转换为2.5高程栅格，并发布为可保存的地图话题
## Build
- Clone
```bash
git clone https://github.com/qza36/COD_PCD2PGM.git ros2_ws/src
cd ros2_ws && rosdep install --from-paths src --ignore-src -y
```
- Build
```
colcon build --symlink-install
source install/setup.bash
```
## Config
- 修改 `config/config.yaml` ,填写正确的pcd地址
- 修改 `config/PclLoderParameters.yaml`,修改点云transform
## Run
```Bash
ros2 launch pcd2pgm pcd2pgm.launch.py
```
保存地图
```Bash
ros2 run nav2_map_server map_saver_cli -f map -t /elevation_grid
```

[LICENSE](LICENSE)

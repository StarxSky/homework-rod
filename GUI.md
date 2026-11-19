已经帮你重新启动好了，现在可以直接在浏览器观察。
1. 打开浏览器 VNC
确认容器在运行：
cd /Users/starxsky/Downloads/workspace/ROS
docker compose up -d gui
然后浏览器（Safari/Chrome）打开：
http://localhost:6080/vnc.html?autoconnect=1&resize=scale
- 不需要密码（本地无密码）
- 如果没自动连上，点页面里的 Connect
- resize=scale 会自动适配窗口
打开后是一个 Linux 桌面，里面有：
- Gazebo GUI：教室 3D 场景 + 小车
- RViz：激光点云 + 障碍物标记（可能被 Gazebo 挡在后面，点底部任务栏切换即可）
2. 让小车跑起来
在 Mac 终端另开一个窗口（不要关 VNC）：
cd /Users/starxsky/Downloads/workspace/ROS
docker compose exec gui bash -lc '
  source /opt/ros/humble/setup.bash
  source /workspace/install/setup.bash
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
'
焦点放在这个终端上，用键盘控制：
- i 前进，, 后退
- j 左转，l 右转
- k 停止
Gazebo 里的小车会动，RViz 里的激光点和障碍物标记会同步变化。也可以单次发速度：
docker compose exec gui bash -lc '
  source /opt/ros/humble/setup.bash
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.3}, angular: {z: 0.0}}"
'
3. 把视角对准小车
# 镜头跟随小车
docker compose exec gui bash -lc 'ign service -s /gui/move_to \
  --reqtype ignition.msgs.StringMsg --reptype ignition.msgs.Boolean \
  --timeout 5000 --req "data: \"classroom_car\""'

# 恢复全景教室
docker compose exec gui bash -lc 'ign service -s /gui/move_to \
  --reqtype ignition.msgs.StringMsg --reptype ignition.msgs.Boolean \
  --timeout 5000 --req "data: \"classroom\""'
4. 也可以不用浏览器
macOS 自带屏幕共享也能连：
- Finder → 前往 → 连接服务器 → vnc://localhost:5900（无密码）
5. 常见问题
- 页面打不开：先看 docker compose ps 是否有 ros2_lidar_gui Up，再看 curl -I http://localhost:6080/vnc.html；必要时 docker compose up -d gui。
- 黑屏/没画面：首次启动等 30–60 秒，刷新页面；软件渲染较慢属正常。
- ros2 topic echo 说找不到话题：执行一次 ros2 daemon stop。
- 停止：docker compose down；下次再用 docker compose up -d gui。
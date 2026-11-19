# 基于 ROS 2 的二维激光雷达模拟与障碍物检测系统

基于 **ROS 2 Humble** + **Gazebo Fortress** 的完整仿真工程：在 Gazebo 中搭建一个带
360° 二维激光雷达的差速驱动机器人和一个布满障碍物的场地，通过 `ros_gz_bridge` 把
`LaserScan` 引入 ROS 2，再用**距离间隙聚类**算法检测障碍物，并输出自定义消息、
RViz 可视化标记以及最近障碍物距离告警。

- 机器人：差速驱动 + 云台激光雷达，可在场地内遥控移动
- 传感器：`gpu_lidar`，720 线/圈、10 Hz、量程 0.12–12 m
- 检测：按相邻扫描点的欧氏距离聚类，输出质心、距离、方位、宽度、点数
- 可视化：RViz 中显示机器人、点云扫描、障碍物圆柱标记与距离标签
- 一键启动：`ros2 launch lidar_sim_bringup simulation.launch.py`

---

## 目录结构

```
ROS/
├── docker-compose.yml            # Docker 运行编排
├── docker/
│   ├── Dockerfile                # ROS 2 Humble + Gazebo Fortress 镜像
│   └── entrypoint.sh             # 自动 source 工作空间
├── blender/
│   └── build_classroom_scene.py  # 程序化生成教室 + 小车并导出 ROS 资源
├── src/
│   ├── lidar_robot_description/  # 机器人模型（xacro）
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── urdf/lidar_robot.urdf.xacro
│   ├── lidar_sim_bringup/        # 仿真启动与配置
│   │   ├── launch/simulation.launch.py
│   │   ├── worlds/obstacle_arena.sdf
│   │   ├── config/gz_bridge.yaml
│   │   └── rviz/obstacle_detection.rviz
│   ├── classroom_sim/            # Blender 教室场景 + 小车
│   │   ├── launch/classroom.launch.py
│   │   ├── worlds/classroom.sdf
│   │   ├── urdf/classroom_car.urdf.xacro
│   │   ├── config/gz_bridge.yaml
│   │   ├── rviz/classroom.rviz
│   │   └── models/{classroom,classroom_car}/
│   ├── obstacle_detector_msgs/   # 自定义消息
│   │   └── msg/{Obstacle,ObstacleArray}.msg
│   └── obstacle_detector/        # 检测节点（Python）
│       ├── obstacle_detector/detector_node.py
│       ├── obstacle_detector/odom_tf_broadcaster.py
│       ├── config/detector_params.yaml
│       └── launch/detector.launch.py
└── README.md
```

---

## 系统架构

```
                    Gazebo Fortress
   ┌──────────────────────────────────────────────┐
   │  gpu_lidar ──/scan──┐                         │
   │  DiffDrive  ◀─cmd_vel（ROS→GZ）               │
   │      │odometry      │                         │
   └──────┼──────────────┼─────────────────────────┘
          │              │
     ros_gz_bridge       │
          │              │
   /clock /cmd_vel       │ /scan（frame_id 修正为 laser_link）
   /odom                 │
          │              ▼
          │        obstacle_detector（聚类）
          │              │
          │       ┌──────┴───────────────────────────┐
          │       │ /obstacles          ObstacleArray │
          │       │ /obstacles/markers  MarkerArray   │
          │       │ /nearest_obstacle_distance Float32│
          │       └───────────────────────────────────┘
          │
   odom_tf_broadcaster ── TF: odom → base_link
```

`robot_state_publisher` 根据 URDF 发布 `base_link` 到各连杆（`laser_link`、车轮等）的
TF；`odom_tf_broadcaster` 把 `/odom` 转换为 `odom → base_link` 的 TF，从而在 RViz 中
固定坐标系 `odom` 下正确显示机器人运动。

---

## 环境要求

| 方式 | 说明 |
|---|---|
| **推荐：Docker** | Docker Desktop（macOS/Windows）或 Docker Engine（Linux），镜像基于 `ros:humble-desktop` |
| 原生构建 | Ubuntu 22.04 + ROS 2 Humble + `ros-humble-ros-gz`（Gazebo Fortress） |

> `ros-humble-ros-gz` 会自动安装 Humble 官方配套的 Gazebo Fortress。

---

## 快速开始（Docker）

在仓库根目录执行：

```bash
# 1. 构建镜像（首次会编译工作空间，耗时较长）
docker compose build

# 2. 启动交互式容器
docker compose run --rm sim
```

进入容器后（镜像已自动编译并 source `install/setup.bash`）：

```bash
# 一键启动：Gazebo + 机器人 + bridge + 检测 + RViz
ros2 launch lidar_sim_bringup simulation.launch.py
```

在**另一个终端**进入同一容器遥控机器人：

```bash
docker compose exec sim bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

查看检测结果：

```bash
ros2 topic echo /obstacles --once              # 障碍物列表（自定义消息）
ros2 topic echo /nearest_obstacle_distance     # 最近障碍物距离
ros2 topic hz /scan                            # 激光数据频率（约 10 Hz）
```

### 只启动检测（假设已有 `/scan`）

```bash
ros2 launch obstacle_detector detector.launch.py use_sim_time:=false
```

---

## macOS 图形界面（XQuartz）

> 更省事的方式是用 `docker compose up -d gui` 走浏览器 VNC，
> 见“教室场景 → 浏览器图形界面（noVNC）”，无需 XQuartz。
> 下面的 XQuartz 方案适合希望把 Gazebo/RViz 作为 macOS 原生 X11 窗口使用的场景。

Gazebo 与 RViz 都是 OpenGL/Qt 程序，Docker on macOS 需要 X 服务器：

1. 安装并启动 **XQuartz**。
2. 打开 XQuartz → 设置 → 安全性，勾选 **允许网络客户端连接**，然后重启 XQuartz。
3. 允许 Docker 主机连接：
   ```bash
   xhost + 127.0.0.1
   ```
4. `docker-compose.yml` 默认 `DISPLAY=host.docker.internal:0`；如需手动指定：
   ```bash
   DISPLAY=host.docker.internal:0 docker compose run --rm sim
   ```
5. macOS 下容器没有 GPU 直通，采用软件渲染（速度较慢）。若 Gazebo 无法创建渲染
   上下文，尝试：
   ```bash
   LIBGL_ALWAYS_SOFTWARE=1 docker compose run --rm sim
   ```

> 追求流畅体验建议在 **Linux 主机**（可用 `/dev/dri` GPU 直通）或 Ubuntu 虚拟机中运行。

Linux 主机可直接：

```bash
export DISPLAY=:0
xhost +local:docker
docker compose run --rm sim
# 如需 GPU 加速，取消 docker-compose.yml 中 /dev/dri 的注释
```

### 无界面运行

```bash
ros2 launch lidar_sim_bringup simulation.launch.py use_rviz:=false
```

---

## 原生构建（Ubuntu 22.04）

```bash
sudo apt update
sudo apt install -y \
  ros-humble-ros-gz ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-rviz2 ros-humble-teleop-twist-keyboard \
  python3-colcon-common-extensions

cd ROS
colcon build --symlink-install
source install/setup.bash
ros2 launch lidar_sim_bringup simulation.launch.py
```

---

## 障碍物检测算法

检测节点订阅 `/scan`，对每一帧扫描执行以下步骤：

1. **过滤无效点**：丢弃 `inf/NaN` 以及不在 `[min_range, max_range]` 内的返回；
2. **极坐标转直角坐标**：以激光雷达为原点计算每个点的 `(x, y)`；
3. **间隙聚类**：按扫描顺序遍历相邻有效点，若两点欧氏距离大于
   `cluster_distance`（或扫描索引跳跃过大），则断开并开始新簇；
4. **噪声过滤**：丢弃点数少于 `min_cluster_points` 的簇；
5. **特征计算**：对每个簇求质心、到雷达的距离、方位角、近似宽度和点数；
6. **发布**：`ObstacleArray`、RViz `MarkerArray`、最近距离 `Float32`，并在距离小于
   `warning_distance` 时打印告警日志。

### 参数配置

编辑 `src/obstacle_detector/config/detector_params.yaml`：

| 参数 | 默认值 | 含义 |
|---|---|---|
| `cluster_distance` | `0.30` m | 同一障碍物相邻点的最大间距，越大越容易把多个物体合并 |
| `min_cluster_points` | `5` | 一个簇的最少点数，用于抑制噪声 |
| `min_range` / `max_range` | `0.12` / `12.0` m | 有效激光量程 |
| `warning_distance` | `1.0` m | 最近障碍物告警阈值 |
| `output_frame` | `laser_link` | 障碍物与标记使用的坐标系 |
| `marker_lifetime` | `0.5` s | RViz 标记存活时间 |

修改后重新启动 launch 即可生效。

### 自定义消息

`Obstacle.msg`

```text
std_msgs/Header header
int32 id
geometry_msgs/Point position
float32 distance
float32 angle
float32 width
uint32 point_count
```

`ObstacleArray.msg`

```text
std_msgs/Header header
Obstacle[] obstacles
```

---

## 话题一览

| 话题 | 类型 | 方向/来源 |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | Gazebo → ROS |
| `/cmd_vel` | `geometry_msgs/Twist` | ROS → Gazebo |
| `/odom` | `nav_msgs/Odometry` | Gazebo → ROS |
| `/clock` | `rosgraph_msgs/Clock` | Gazebo → ROS |
| `/obstacles` | `obstacle_detector_msgs/ObstacleArray` | 检测节点输出 |
| `/obstacles/markers` | `visualization_msgs/MarkerArray` | 检测节点输出（RViz） |
| `/nearest_obstacle_distance` | `std_msgs/Float32` | 检测节点输出（无目标时为 -1） |

### 关键 TF

```
odom ──(odom_tf_broadcaster)──▶ base_link ──(robot_state_publisher)──▶ laser_link / wheels
```

---

## launch 参数

`simulation.launch.py` 支持：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `use_sim_time` | `true` | 使用 Gazebo 仿真时钟 |
| `use_rviz` | `true` | 是否启动 RViz |
| `gz_args` | `-r -v 3 <world>` | 传给 Gazebo 的参数，可替换世界文件 |

示例：更换世界文件

```bash
ros2 launch lidar_sim_bringup simulation.launch.py \
  gz_args:="-r -v 3 /path/to/your_world.sdf"
```

---

## 教室场景（Blender 生成）

`classroom_sim` 包提供一套用 Blender 程序化生成的教室场景，以及一辆可在其中行驶的
四轮差速小车（车顶带 360° 激光雷达）：

- 房间：8 × 6 × 3 m，含黑板、窗户、门、天花板灯、6 张课桌 + 12 把椅子、
  讲台与储物柜；
- 小车：Blender 导出的车身/车窗/车灯/车轮网格，后轮驱动、前轮被动滚动；
- 碰撞体由同一套布局参数生成，因此 Gazebo 里的激光雷达能“看到”渲染出的家具。

### 重新生成场景

Blender 脚本是场景的唯一数据源，会同时导出 OBJ 网格并写出 `model.sdf` 和
`model.config`（修改布局后重新运行即可）：

```bash
blender --background --python blender/build_classroom_scene.py
```

生成物：

```
blender/classroom_scene.blend                       # 可在 Blender 中继续编辑
src/classroom_sim/models/classroom/meshes/*.obj     # 教室网格
src/classroom_sim/models/classroom/model.sdf        # 视觉 + 碰撞体
src/classroom_sim/models/classroom_car/meshes/*.obj # 小车网格（Gazebo）
src/classroom_sim/models/classroom_car/meshes/*.stl # 小车网格（RViz）
```

### 运行教室仿真

重新构建镜像/工作空间后启动：

```bash
# Docker（在仓库根目录）
docker compose build
docker compose run --rm sim
ros2 launch classroom_sim classroom.launch.py

# 原生
colcon build --symlink-install && source install/setup.bash
ros2 launch classroom_sim classroom.launch.py
```

小车出生在教室右侧过道 `(2.9, 2.55)`，朝向黑板。话题与主场景一致
（`/scan`、`/cmd_vel`、`/odom`、`/obstacles`），可继续用
`ros2 run teleop_twist_keyboard teleop_twist_keyboard` 遥控。Gazebo 侧话题前缀为
`/model/classroom_car/...`，launch 会自动把 `models/` 加入
`IGN_GAZEBO_RESOURCE_PATH` / `GZ_SIM_RESOURCE_PATH` 以便解析 `model://classroom`。

> 该 launch 默认让 Gazebo **服务端无界面运行**，再单独启动 Gazebo GUI
> （`use_gz_gui:=false` 可关闭），避免 `ign gazebo` 在纯软件渲染下卡死。

### 浏览器图形界面（noVNC，macOS 无需 XQuartz）

`gui` 服务在容器内启动 `Xvfb + x11vnc + noVNC`，自动拉起 Gazebo 服务端、
Gazebo GUI、RViz、bridge 和检测节点。无需 XQuartz，也不需要管理员权限。

```bash
cd ROS
docker compose up -d gui
```

#### 1. 连接 VNC

浏览器（Safari/Chrome）打开：

```
http://localhost:6080/vnc.html?autoconnect=1&resize=scale
```

- 不需要密码；没有自动连接时点页面里的 **Connect**。
- `autoconnect=1` 自动连接，`resize=scale` 随窗口缩放。
- 首次启动约 30–60 秒（Apple Silicon 为软件渲染，刷新会慢一些）；
  Gazebo 视角会自动移动到教室。
- 桌面里有 **Gazebo GUI**（教室 + 小车）和 **RViz**（激光点云 + 障碍物标记）。
  两个窗口可能重叠，用底部任务栏或 `Alt+Tab` 切换。

也可以不用浏览器，用 macOS 自带屏幕共享：
Finder → 前往 → 连接服务器 → `vnc://localhost:5900`（无密码）。

#### 2. 遥控小车

在宿主机另开一个终端（保持 VNC 页面打开）：

```bash
cd ROS
docker compose exec gui bash -lc '
  source /opt/ros/humble/setup.bash
  source /workspace/install/setup.bash
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
'
```

按键：`i` 前进、`,` 后退、`j` 左转、`l` 右转、`k` 停止。Gazebo 中的小车会移动，
RViz 中的 `/scan` 与 `/obstacles/markers` 同步更新。

也可以只发一次速度指令：

```bash
docker compose exec gui bash -lc '
  source /opt/ros/humble/setup.bash
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.3}, angular: {z: 0.0}}"
'
```

#### 3. 调整 Gazebo 视角

```bash
# 镜头对准小车
docker compose exec gui bash -lc 'ign service -s /gui/move_to \
  --reqtype ignition.msgs.StringMsg --reptype ignition.msgs.Boolean \
  --timeout 5000 --req "data: \"classroom_car\""'

# 恢复教室全景
docker compose exec gui bash -lc 'ign service -s /gui/move_to \
  --reqtype ignition.msgs.StringMsg --reptype ignition.msgs.Boolean \
  --timeout 5000 --req "data: \"classroom\""'
```

#### 4. 常用运维

```bash
docker compose ps                 # 查看 gui 容器状态
docker compose logs -f gui        # 查看 launch / Gazebo / RViz 日志
docker compose down               # 停止（VNC + 仿真）
docker compose up -d gui          # 再次启动
```

- `gui` 服务带 `restart: unless-stopped`，进程意外退出会自动拉起。
- 需要容器内终端时：`docker compose exec gui bash`
- 只跑无界面服务端：
  `ros2 launch classroom_sim classroom.launch.py use_gz_gui:=false use_rviz:=false`
- 若 `ros2 topic echo` 提示 “not published / unknown topic”，先执行一次
  `ros2 daemon stop`（ROS 2 daemon 缓存问题）。
- 从局域网其他设备访问：把 `localhost` 换成 Mac 的 IP，并放行 `6080` 端口。
  noVNC 默认无密码，公网环境请自行加密码/防火墙。
- Apple Silicon 软件渲染较慢，可把 `docker/start_gui.sh` 中
  `1600x1000x24` 改为 `1280x800x24` 后重建镜像。

---

## 常见问题

- **收不到 `/scan`**
  确认 Gazebo 已加载 `Sensors` 系统插件且渲染引擎为 `ogre2`。无 GPU/EGL 的纯无头
  容器可能无法生成激光数据，建议使用 GUI 或带 GPU 的 Linux 环境。

- **RViz 提示 “No transform / Fixed Frame”**
  固定坐标系为 `odom`。确认 `odom_tf_broadcaster` 正在运行，且 `/odom` 有数据：
  `ros2 topic echo /odom --once`。

- **机器人不动**
  遥控器发布的是 `/cmd_vel`，确认 bridge 正常，并检查 Gazebo 侧话题：
  `ign topic -l | grep cmd_vel`。

- **Spawn 报 “entity already exists”**
  同一个 Gazebo 世界重复生成同名模型，重启容器即可清理。

- **Gazebo 打不开图形界面（macOS）**
  检查 XQuartz 是否允许网络客户端、`DISPLAY` 是否正确、是否设置
  `xhost + 127.0.0.1`；必要时启用 `LIBGL_ALWAYS_SOFTWARE=1`。

---

## 可扩展方向

- 用 DBSCAN / 欧氏聚类（PCL）替换简单间隙聚类，提升复杂场景鲁棒性
- 加入障碍物跟踪（卡尔曼滤波 / 数据关联）与速度估计
- 结合 Nav2 做避障规划与自主导航
- 通过 `ros2_control` 替换 Gazebo DiffDrive 插件，统一控制接口
- 增加多机器人命名空间与多车协同

---

## 许可证

Apache-2.0

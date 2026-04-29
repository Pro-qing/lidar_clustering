这是一份为你量身定制的、专业且结构清晰的 `README.md` 模板。你可以直接将它复制并保存到 `lidar_clustering` 包的根目录下。

这份文档不仅包含了项目的说明、使用方法，还特别突出了你的**热更新亮点**，并为你未来的开发进度（Roadmap）做好了规划。

---

# 📦 Lidar Clustering (3D激光雷达点云聚类与包围框生成)

![ROS Version](https://img.shields.io/badge/ROS-Noetic%20%7C%20Melodic-blue)
![Build](https://img.shields.io/badge/Build-colcon-brightgreen)
![Version](https://img.shields.io/badge/Version-v1.0.0-orange)
![License](https://img.shields.io/badge/License-MIT-green)

这是一个用于自动驾驶和移动机器人的 3D 激光雷达点云聚类 ROS 功能包。该包接收前处理后的干净点云，基于 PCL 的欧式聚类算法（Euclidean Cluster Extraction）提取独立障碍物，并生成 3D 轴对齐包围盒（AABB），最后通过 RViz 进行可视化发布。

**🔥 核心亮点**：支持基于 `inotify` 机制的 **YAML 参数热更新**，在调车和部署时修改聚类参数 **无需重启节点**，即可瞬间生效。

---

## 🚀 1. 核心功能特性 (Features)
- 基于 PCL KD-Tree 加速的高效欧式聚类。
- AABB (Axis-Aligned Bounding Box) 3D 包围盒生成。
- **动态热更新**：采用 Linux 原生 `inotify` 监听 YAML 文件，保存即生效。
- **固定绝对路径**：专为工控机/车载设备部署设计，避免相对路径带来的环境变量解析问题。
- 轻量化多线程锁设计，保证点云回调与参数加载的线程安全。

---

## 🛠️ 2. 环境依赖 (Dependencies)
- **ROS 1** (Ubuntu 18.04 Melodic / Ubuntu 20.04 Noetic)
- **PCL** (Point Cloud Library)
- **yaml-cpp** (用于解析 YAML 配置文件)

安装必要的依赖：
```bash
sudo apt update
sudo apt install libyaml-cpp-dev ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-pcl-conversions
```

---

## 🏗️ 3. 编译与安装 (Build)

推荐使用 `colcon build` 进行编译：

```bash
# 进入你的工作空间
cd ~/lidar_filtering

# 编译指定的包
colcon build --packages-select lidar_clustering

# 刷新环境变量
source install/setup.bash
```

---

## 🏃 4. 运行与使用 (Usage)

启动聚类节点：
```bash
roslaunch lidar_clustering clustering.launch
```

**可视化：**
打开 `RViz`，添加 `MarkerArray` 插件，并将 Topic 设置为 `/cluster_bounding_boxes`，即可看到绿色半透明的 3D 包围框。

---

## ⚙️ 5. 参数配置与热更新 (Configuration & Hot-Reload)

本包强制使用**绝对路径**读取配置文件，默认路径配置在 `clustering.launch` 中：
```xml
<arg name="config_path" default="/home/getq/lidar_filtering/src/lidar_clustering/params/lidar_cluster.yaml" />
```

### 参数说明 (`lidar_cluster.yaml`)
```yaml
# 聚类容忍度 (米)：点与点之间距离小于该值，视为同一个物体
cluster_tolerance: 0.5

# 聚类最小点数：少于该点数的聚类将被视为噪点丢弃
min_cluster_size: 10

# 聚类最大点数：大于该点数的聚类将被截断或丢弃
max_cluster_size: 5000

# 调试模式：开启后终端将每秒打印检测耗时与包围框数量
debug_mode: true
```

💡 **热更新测试：**
节点运行期间，直接打开上述 yaml 文件，修改 `cluster_tolerance` 并保存。终端会立即提示 `[Clustering] Loaded config!`，RViz 中的包围框将实时改变，无需 `Ctrl+C` 重启节点。

---

## 📡 6. 话题说明 (Nodes & Topics)

### 订阅的话题 (Subscribed Topics)
* `/points_filter` (`sensor_msgs/PointCloud2`) : 前处理和地面过滤后的干净点云。

### 发布的话题 (Published Topics)
* `/cluster_bounding_boxes` (`visualization_msgs/MarkerArray`) : 用于 RViz 可视化的 3D 包围框。

---

## 📊 7. 版本记录与开发进度 (Roadmap)

### 当前版本：`v1.0.0`
- [y] 完成基础 PCL 欧式聚类与 AABB 包围框生成。
- [y] 完成基于 `yaml-cpp` 的配置抽离。
- [y] 实现基于 `inotify` 的绝对路径 YAML 动态热更新。
- [y] 添加 Launch 文件和标准包结构。

### 📅 下一步开发计划 (TODO / Work In Progress)
- [ ] **OBB 旋转包围盒**：引入 PCA (主成分分析)，将 AABB 升级为贴合车辆姿态的有向包围盒（Oriented Bounding Box）。
- [ ] **规划接口输出**：不仅发布 Marker，额外发布标准的感知结果消息（如 `autoware_msgs/DetectedObjectArray` 或 `jsk_recognition_msgs/BoundingBoxArray`），供下游局部路径规划使用。
- [ ] **障碍物 ID 跟踪 (Tracking)**：引入卡尔曼滤波 (KF) 或简单的匈牙利匹配算法，为上一帧和下一帧的同一个包围框赋予一致的 ID 并计算速度。
- [ ] **多边形拟合提取 (Convex Hull)**：为特大障碍物（如花坛、墙壁）生成多边形轮廓而非简单的方形包围盒。
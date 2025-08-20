# RTAB-Map传感器与ElevationMapping集成使用指南

## 概述

这个集成方案将RTAB-Map的传感器配置和ICP处理功能与ElevationMapping的高程建图功能相结合，提供更精确的2.5D高程地图构建。

## 主要特性

### 1. 传感器配置
- **RealSense D435i** 深度相机
- 支持点云、RGB、深度、IMU数据
- 自动TF发布和同步

### 2. ICP里程计
- 基于RTAB-Map的ICP算法
- 点云配准和位姿估计
- 支持去偏斜（deskewing）

### 3. 点云处理
- 点云组装器（Point Cloud Assembler）
- 多帧点云融合
- 噪声过滤和优化

### 4. 高程建图
- 基于ElevationMapping的2.5D地图构建
- 卡尔曼滤波融合
- 后处理和可视化

## 文件结构

```
launch/
├── rtabmap_sensor_elevation_mapping.launch.py  # 主启动文件
config/
├── rtabmap_sensor_elevation_mapping.param.yaml # 集成配置文件
docs/
└── rtabmap_sensor_integration.md              # 本文档
```

## 使用方法

### 1. 启动系统

```bash
# 启动集成系统
ros2 launch elevation_mapping_ros2 rtabmap_sensor_elevation_mapping.launch.py

# 可选参数
ros2 launch elevation_mapping_ros2 rtabmap_sensor_elevation_mapping.launch.py \
    use_sim_time:=false \
    deskewing:=false \
    use_rtabmapviz:=true
```

### 2. 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_sim_time` | false | 是否使用仿真时间 |
| `deskewing` | false | 是否启用激光雷达去偏斜 |
| `use_rtabmapviz` | true | 是否启动RTAB-Map可视化 |

### 3. 主要Topics

#### 输入Topics
- `/camera/camera/depth/color/points` - D435i点云数据
- `/camera/camera/infra1/image_rect_raw` - 红外图像
- `/camera/camera/depth/image_rect_raw` - 深度图像
- `/imu/data` - IMU数据

#### 输出Topics
- `/odom` - ICP里程计位姿
- `raw_elevation_map` - 原始高程地图
- `filtered_map` - 滤波后高程地图
- `assembled_cloud` - 组装后的点云

### 4. 坐标系

```
map -> camera_link -> camera_depth_optical_frame
```

- `map`: 全局地图坐标系
- `camera_link`: 机器人坐标系
- `camera_depth_optical_frame`: 传感器坐标系

## 配置优化

### 1. 传感器参数调整

在 `config/rtabmap_sensor_elevation_mapping.param.yaml` 中：

```yaml
sensor:
  cutoff_min_depth: 0.2    # 最小深度阈值
  cutoff_max_depth: 4.0    # 最大深度阈值（增大范围）
  lateral_factor: 0.01576  # 横向方差因子
```

### 2. ICP参数优化

在launch文件中调整ICP参数：

```python
arguments=[
    'Icp/PointToPlane', 'true',
    'Icp/Iterations', '10',           # 迭代次数
    'Icp/VoxelSize', '0.1',           # 体素大小
    'Icp/MaxTranslation', '2',        # 最大平移
    'Icp/MaxCorrespondenceDistance', '1', # 最大对应距离
]
```

### 3. 地图参数

```yaml
grid_map:
  lateral_length: 15.0      # 地图横向尺寸
  longitudinal_length: 15.0 # 地图纵向尺寸
  resolution: 0.05          # 分辨率
```

## 故障排除

### 1. TF错误
确保所有必要的TF都已发布：
```bash
ros2 run tf2_tools view_frames
```

### 2. 点云数据问题
检查点云话题：
```bash
ros2 topic echo /camera/camera/depth/color/points
```

### 3. 位姿估计问题
检查ICP里程计输出：
```bash
ros2 topic echo /odom
```

## 性能优化建议

1. **降低分辨率**：如果性能不足，可以降低深度图像分辨率
2. **调整ICP参数**：根据环境复杂度调整ICP迭代次数
3. **优化地图大小**：根据实际需求调整地图尺寸
4. **启用去偏斜**：在快速运动时启用deskewing

## 可视化

### 1. RViz2配置
加载 `rviz2/rviz2.rviz` 配置文件查看高程地图

### 2. RTAB-Map可视化
启用 `use_rtabmapviz:=true` 查看RTAB-Map的可视化界面

### 3. 实时监控
```bash
# 查看高程地图
ros2 topic echo raw_elevation_map

# 查看点云
ros2 topic echo assembled_cloud
```

## 扩展功能

1. **多传感器融合**：可以添加激光雷达等其他传感器
2. **SLAM集成**：可以集成其他SLAM算法
3. **路径规划**：基于高程地图进行路径规划
4. **障碍物检测**：利用高程信息进行障碍物检测 
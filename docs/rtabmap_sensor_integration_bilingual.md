# RTAB-Map传感器与ElevationMapping集成使用指南
# RTAB-Map Sensor and ElevationMapping Integration User Guide

## 概述 / Overview

这个集成方案将RTAB-Map的传感器配置和ICP处理功能与ElevationMapping的高程建图功能相结合，提供更精确的2.5D高程地图构建。

This integration combines RTAB-Map's sensor configuration and ICP processing capabilities with ElevationMapping's elevation mapping functionality to provide more accurate 2.5D elevation map construction.

## 主要特性 / Main Features

### 1. 传感器配置 / Sensor Configuration
- **RealSense D435i** 深度相机 / **RealSense D435i** Depth Camera
- 支持点云、RGB、深度、IMU数据 / Supports point cloud, RGB, depth, and IMU data
- 自动TF发布和同步 / Automatic TF publishing and synchronization

### 2. ICP里程计 / ICP Odometry
- 基于RTAB-Map的ICP算法 / ICP algorithm based on RTAB-Map
- 点云配准和位姿估计 / Point cloud registration and pose estimation
- 支持去偏斜（deskewing）/ Supports deskewing

### 3. 点云处理 / Point Cloud Processing
- 点云组装器（Point Cloud Assembler）/ Point Cloud Assembler
- 多帧点云融合 / Multi-frame point cloud fusion
- 噪声过滤和优化 / Noise filtering and optimization

### 4. 高程建图 / Elevation Mapping
- 基于ElevationMapping的2.5D地图构建 / 2.5D map construction based on ElevationMapping
- 卡尔曼滤波融合 / Kalman filter fusion
- 后处理和可视化 / Post-processing and visualization

## 文件结构 / File Structure

```
launch/
├── rtabmap_sensor_elevation_mapping.launch.py  # 主启动文件 / Main launch file
config/
├── rtabmap_sensor_elevation_mapping.param.yaml # 集成配置文件 / Integration config file
docs/
└── rtabmap_sensor_integration.md              # 本文档 / This document
```

## 使用方法 / Usage

### 1. 启动系统 / Launch System

```bash
# 启动集成系统 / Launch integrated system
ros2 launch elevation_mapping_ros2 rtabmap_sensor_elevation_mapping.launch.py

# 可选参数 / Optional parameters
ros2 launch elevation_mapping_ros2 rtabmap_sensor_elevation_mapping.launch.py \
    use_sim_time:=false \
    deskewing:=false \
    use_rtabmapviz:=true
```

### 2. 参数说明 / Parameter Description

| 参数 / Parameter | 默认值 / Default | 说明 / Description |
|------------------|------------------|-------------------|
| `use_sim_time` | false | 是否使用仿真时间 / Whether to use simulation time |
| `deskewing` | false | 是否启用激光雷达去偏斜 / Whether to enable lidar deskewing |
| `use_rtabmapviz` | true | 是否启动RTAB-Map可视化 / Whether to start RTAB-Map visualization |

### 3. 主要Topics / Main Topics

#### 输入Topics / Input Topics
- `/camera/camera/depth/color/points` - D435i点云数据 / D435i point cloud data
- `/camera/camera/infra1/image_rect_raw` - 红外图像 / Infrared image
- `/camera/camera/depth/image_rect_raw` - 深度图像 / Depth image
- `/imu/data` - IMU数据 / IMU data

#### 输出Topics / Output Topics
- `/odom` - ICP里程计位姿 / ICP odometry pose
- `raw_elevation_map` - 原始高程地图 / Raw elevation map
- `filtered_map` - 滤波后高程地图 / Filtered elevation map
- `assembled_cloud` - 组装后的点云 / Assembled point cloud

### 4. 坐标系 / Coordinate Frames

```
map -> camera_link -> camera_depth_optical_frame
```

- `map`: 全局地图坐标系 / Global map coordinate frame
- `camera_link`: 机器人坐标系 / Robot coordinate frame
- `camera_depth_optical_frame`: 传感器坐标系 / Sensor coordinate frame

## 配置优化 / Configuration Optimization

### 1. 传感器参数调整 / Sensor Parameter Adjustment

在 `config/rtabmap_sensor_elevation_mapping.param.yaml` 中 / In `config/rtabmap_sensor_elevation_mapping.param.yaml`:

```yaml
sensor:
  cutoff_min_depth: 0.2    # 最小深度阈值 / Minimum depth threshold
  cutoff_max_depth: 4.0    # 最大深度阈值（增大范围）/ Maximum depth threshold (increased range)
  lateral_factor: 0.01576  # 横向方差因子 / Lateral variance factor
```

### 2. ICP参数优化 / ICP Parameter Optimization

在launch文件中调整ICP参数 / Adjust ICP parameters in launch file:

```python
arguments=[
    'Icp/PointToPlane', 'true',
    'Icp/Iterations', '10',           # 迭代次数 / Number of iterations
    'Icp/VoxelSize', '0.1',           # 体素大小 / Voxel size
    'Icp/MaxTranslation', '2',        # 最大平移 / Maximum translation
    'Icp/MaxCorrespondenceDistance', '1', # 最大对应距离 / Maximum correspondence distance
]
```

### 3. 地图参数 / Map Parameters

```yaml
grid_map:
  lateral_length: 15.0      # 地图横向尺寸 / Map lateral size
  longitudinal_length: 15.0 # 地图纵向尺寸 / Map longitudinal size
  resolution: 0.05          # 分辨率 / Resolution
```

## 故障排除 / Troubleshooting

### 1. TF错误 / TF Errors
确保所有必要的TF都已发布 / Ensure all necessary TFs are published:
```bash
ros2 run tf2_tools view_frames
```

### 2. 点云数据问题 / Point Cloud Data Issues
检查点云话题 / Check point cloud topic:
```bash
ros2 topic echo /camera/camera/depth/color/points
```

### 3. 位姿估计问题 / Pose Estimation Issues
检查ICP里程计输出 / Check ICP odometry output:
```bash
ros2 topic echo /odom
```

## 性能优化建议 / Performance Optimization Suggestions

1. **降低分辨率** / **Reduce Resolution**: 如果性能不足，可以降低深度图像分辨率 / If performance is insufficient, reduce depth image resolution
2. **调整ICP参数** / **Adjust ICP Parameters**: 根据环境复杂度调整ICP迭代次数 / Adjust ICP iterations based on environment complexity
3. **优化地图大小** / **Optimize Map Size**: 根据实际需求调整地图尺寸 / Adjust map size based on actual requirements
4. **启用去偏斜** / **Enable Deskewing**: 在快速运动时启用deskewing / Enable deskewing during fast motion

## 可视化 / Visualization

### 1. RViz2配置 / RViz2 Configuration
加载 `rviz2/rviz2.rviz` 配置文件查看高程地图 / Load `rviz2/rviz2.rviz` config file to view elevation map

### 2. RTAB-Map可视化 / RTAB-Map Visualization
启用 `use_rtabmapviz:=true` 查看RTAB-Map的可视化界面 / Enable `use_rtabmapviz:=true` to view RTAB-Map visualization interface

### 3. 实时监控 / Real-time Monitoring
```bash
# 查看高程地图 / View elevation map
ros2 topic echo raw_elevation_map

# 查看点云 / View point cloud
ros2 topic echo assembled_cloud
```

## 扩展功能 / Extended Features

1. **多传感器融合** / **Multi-sensor Fusion**: 可以添加激光雷达等其他传感器 / Can add other sensors like LiDAR
2. **SLAM集成** / **SLAM Integration**: 可以集成其他SLAM算法 / Can integrate other SLAM algorithms
3. **路径规划** / **Path Planning**: 基于高程地图进行路径规划 / Path planning based on elevation map
4. **障碍物检测** / **Obstacle Detection**: 利用高程信息进行障碍物检测 / Obstacle detection using elevation information

## 常见问题 / FAQ

### Q: 为什么ICP里程计一直失败？ / Why does ICP odometry keep failing?
**A**: 检查点云质量和TF变换是否正确。确保相机固定且环境有足够的特征点。
**A**: Check point cloud quality and TF transformations. Ensure camera is fixed and environment has sufficient feature points.

### Q: 如何提高建图精度？ / How to improve mapping accuracy?
**A**: 调整传感器参数，降低噪声，优化ICP参数，使用更高质量的传感器。
**A**: Adjust sensor parameters, reduce noise, optimize ICP parameters, use higher quality sensors.

### Q: 系统运行缓慢怎么办？ / What to do if the system runs slowly?
**A**: 降低点云分辨率，减少地图大小，优化ICP迭代次数，使用更强大的硬件。
**A**: Reduce point cloud resolution, decrease map size, optimize ICP iterations, use more powerful hardware.

## 技术支持 / Technical Support

如有问题，请参考以下资源 / For issues, please refer to the following resources:

- [RTAB-Map官方文档](http://wiki.ros.org/rtabmap_ros) / [RTAB-Map Official Documentation](http://wiki.ros.org/rtabmap_ros)
- [ElevationMapping官方文档](https://github.com/ANYbotics/elevation_mapping) / [ElevationMapping Official Documentation](https://github.com/ANYbotics/elevation_mapping)
- [RealSense SDK文档](https://github.com/IntelRealSense/realsense-ros) / [RealSense SDK Documentation](https://github.com/IntelRealSense/realsense-ros)

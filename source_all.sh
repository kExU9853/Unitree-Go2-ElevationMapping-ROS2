 #!/bin/bash
source /opt/ros/foxy/setup.bash
# souce all workspace
source /home/xk/unitree_ros2/setup.sh
echo "== source all workspace =="

# set workspace path
REALSENSE_WS="$HOME/realsense_ros_ws"
RTABMAP_WS="$HOME/rtab_ws"
ELEVATION_WS="$HOME/elevation_map_foxy_ws"

echo "1. load RealSense workspace: $REALSENSE_WS"  
source "$REALSENSE_WS/install/setup.bash"

echo "2. load RTAB-Map workspace: $RTABMAP_WS"
source "$RTABMAP_WS/install/setup.bash"

echo "3. load ElevationMapping workspace: $ELEVATION_WS"
source "$ELEVATION_WS/install/setup.bash"

echo "4. verify package is available..."
REALSENSE_PKG=$(ros2 pkg list | grep realsense2_camera)
RTABMAP_PKG=$(ros2 pkg list | grep rtabmap)
ELEVATION_PKG=$(ros2 pkg list | grep elevation_mapping_ros2)

echo "   RealSense package: $REALSENSE_PKG"   
echo "   RTAB-Map package: $RTABMAP_PKG"
echo "   ElevationMapping package: $ELEVATION_PKG"

echo "== all workspace loaded =="
echo "now you can use the following command to start the system:"
echo "# disable rtabmapviz launch ros2 launch elevation_mapping_ros2 rtabmap_sensor_elevation_mapping.launch.py use_rtabmapviz:=false"
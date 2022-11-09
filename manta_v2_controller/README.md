# manta_v2_controller
This package controls manta_v2 swimming and walking.

Please prepare this package on both the PC side and the robot side.

# Install && Build
If you have not installed the 'ms5837_node' package and 'ballast_control_node' package, run the following command to install and build the packages.

```
cd ~/manta_ws/src
git clone https://github.com/tasada038/ms5837_node.git
git clone https://github.com/tasada038/ballast_control_node.git
cd ~/manta_ws
colcon build
```
# Diana_7_ros2
## 以Diana_7为控制对象的ROS2-humble-Package
| 包名 | 时间 | 说明 |
| ---- | ---- | ---- |
| diana_7_moveit | 2023.11.14 | Diana_7和moveit简单组合的Package |
| diana7_description | 2023.11.14 | Diana_7描述文件的Package |
## 编译
```bash
mkdir -p ~/Diana_7_ros2_ws/src
cd ~/Diana_7_ros2_ws/src
git clone **
cd ..
colcon build 
source ./install/setup.bash
ros2 launch diana_7_moveit demo.launch.py
ros2 launch diana_7_description display.launch.py

```
## 依赖
```bash
sudo apt-get install ros-humble-moveit
sudo apt-get install ros-humble-joint-state-publisher-gui
sudo apt-get install ros-humble-ros-controllers
```
## 示意图






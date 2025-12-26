# CEN449 Autonomous Robot Project

[![ROS2](https://img.shields.io/badge/ROS2-Humble%20%7C%20Galactic-blue)](https://docs.ros.org/)
[![TurtleBot3](https://img.shields.io/badge/Robot-TurtleBot3-orange)](https://emanual.robotis.com/docs/en/platform/turtlebot3/)

A ROS2 project that performs autonomous exploration in unknown environments using SLAM and frontier-based exploration algorithm, running on TurtleBot3.

##  Features

-  **SLAM Mapping:** Real-time map creation with SLAM Toolbox
-  **Autonomous Navigation:** Safe path planning with Nav2 stack
-  **Frontier Exploration:** Automatic exploration of unknown areas
-  **Simulation:** Safe testing environment in Gazebo
-  **Visualization:** Real-time monitoring with RViz2

##  Requirements

### System Requirements
- **Operating System:** Ubuntu 22.04 LTS (Jammy) or Ubuntu 20.04 LTS (Focal)
- **RAM:** Minimum 8 GB (16 GB recommended)
- **Disk Space:** Minimum 20 GB free space
- **GPU:** OpenGL 3.3+ support (for Gazebo)

### Software Requirements
- ROS2 Humble or Galactic
- Gazebo Classic (11.x)
- Python 3.8+

##  Installation

### 1. ROS2 Installation (Humble)

```bash
# Locale settings
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# ROS2 GPG key
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop
```

### 2. Install Dependencies

```bash
# TurtleBot3 packages
sudo apt install ros-humble-turtlebot3*

# Navigation2 and SLAM
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-slam-toolbox

# Gazebo
sudo apt install ros-humble-gazebo-ros-pkgs

# Development tools
sudo apt install python3-colcon-common-extensions python3-rosdep
```

### 3. Prepare Workspace

```bash
# Create project folder
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Copy project (or git clone)
cp -r /path/to/cen449_project/* .

# Check dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install
```

### 4. Set Environment Variables

Add to `.bashrc` file:

```bash
# ROS2 Humble
source /opt/ros/humble/setup.bash

# Workspace
source ~/ros2_ws/install/setup.bash

# TurtleBot3 model
export TURTLEBOT3_MODEL=waffle_pi

# Gazebo models
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
```

##  Usage

**Terminal 1 - Gazebo:**
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch my_autonomous_robot project_master.launch.py
```

**Terminal 2 - Rviz:**
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run rviz2 rviz2 -d src/my_autonomous_robot/config/my_robot.rviz
```

### Terminal 3: Custom Explorer Node (Not Obligatory)

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run my_autonomous_robot explorer_node

```

**Items to display in RViz:**
- `/map` - Generated map
- `/scan` - LiDAR data
- `/explore/frontiers` - Exploration frontiers (frontier markers)
- Robot model and TF tree

##  Project Structure

```
cen449_project/
├── src/
│   ├── my_autonomous_robot/           
│   │   ├── config/
│   │   │   ├── mapper_params_online_async.yaml  
│   │   │   ├── nav2_params.yaml                
│   │   │   └── my_robot.rviz                   
│   │   ├── launch/
│   │   │   ├── project_master.launch.py         
│   │   │   ├── launch_sim.launch.py             
│   │   │   └── rsp.launch.py                    
│   │   ├── my_autonomous_robot/
│   │   │   └── explorer_node.py                 
│   │   ├── urdf/
│   │   │   └── robot.urdf.xacro                 
│   │   ├── worlds/
│   │   │   └── obstacle_world.world             
│   │   ├── package.xml
│   │   └── setup.py
│   └── m-explore-ros2/               
│       ├── explore/                
│       └── map_merge/               
```

## Youtube Link:
https://youtu.be/mZAzNPDXZd8


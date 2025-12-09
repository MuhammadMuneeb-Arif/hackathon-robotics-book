---
title: Appendix
sidebar_label: Appendix
---

# Appendix

## Setup Guides and Troubleshooting

This appendix provides detailed setup guides and troubleshooting information to help you successfully implement the concepts and exercises in the Physical AI & Humanoid Robotics textbook.

## A.1 System Requirements Verification

### Ubuntu 22.04 LTS Installation
Before beginning the installation of robotics software, ensure you have Ubuntu 22.04 LTS installed. Verify your system with:

```bash
lsb_release -a
```

Expected output should show:
```
Distributor ID: Ubuntu
Description:    Ubuntu 22.04.x LTS
Release:        22.04
Codename:       jammy
```

### System Resource Check
Verify your system meets the minimum requirements:

```bash
# Check CPU
lscpu

# Check RAM
free -h

# Check available disk space
df -h

# Check GPU (if NVIDIA)
nvidia-smi
```

## A.2 ROS 2 Humble Installation Guide

### Prerequisites
Ensure your system is up to date:

```bash
sudo apt update && sudo apt upgrade -y
```

### Add ROS 2 Repository
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe

# Add the ROS 2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the repository to your sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
```

### Install ROS 2 Humble Packages
```bash
sudo apt install ros-humble-desktop
sudo apt install python3-rosdep2
```

### Initialize rosdep
```bash
sudo rosdep init
rosdep update
```

### Setup Environment Variables
Add the following lines to your `~/.bashrc` file:

```bash
# ROS 2 Humble Setup
source /opt/ros/humble/setup.bash
```

Then source your bashrc:
```bash
source ~/.bashrc
```

### Verify Installation
```bash
ros2 --version
```

## A.3 Gazebo Installation Guide

### Install Gazebo Garden
ROS 2 Humble is compatible with Gazebo Garden:

```bash
sudo apt install ignition-garden
```

Or install the ROS 2 Gazebo packages:
```bash
sudo apt install ros-humble-gazebo-*
```

### Test Gazebo Installation
```bash
gz sim --version
```

## A.4 NVIDIA Isaac Setup Guide

### Prerequisites
Ensure you have:
- NVIDIA GPU with CUDA support (RTX 4070 Ti or better recommended)
- NVIDIA drivers installed (version 470 or newer)
- CUDA toolkit installed

### Install NVIDIA Drivers
```bash
# Check if NVIDIA GPU is detected
lspci | grep -i nvidia

# Install NVIDIA drivers
sudo apt install nvidia-driver-535
sudo reboot
```

### Verify GPU Setup
```bash
nvidia-smi
```

### Install CUDA Toolkit
```bash
# Add NVIDIA package repositories
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.0-1_all.deb
sudo dpkg -i cuda-keyring_1.0-1_all.deb
sudo apt-get update

# Install CUDA
sudo apt-get install -y cuda-toolkit-12-3
```

### Install Isaac Sim
Follow the official NVIDIA Isaac Sim installation guide:
1. Download Isaac Sim from NVIDIA Developer website
2. Extract and run the installer
3. Verify installation with Isaac Sim launcher

## A.5 Python Environment Setup

### Create Virtual Environment
```bash
# Install venv if not already installed
sudo apt install python3-venv

# Create a virtual environment for robotics projects
python3 -m venv ~/robotics_env

# Activate the environment
source ~/robotics_env/bin/activate

# Upgrade pip
pip install --upgrade pip
```

### Install Required Python Packages
```bash
# Activate your virtual environment first
source ~/robotics_env/bin/activate

# Install common robotics packages
pip install numpy scipy matplotlib
pip install opencv-python
pip install transforms3d  # For rotation transformations
pip install pygame        # For visualization
```

## A.6 Workspace Setup

### Create ROS 2 Workspace
```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Build the workspace (initially empty)
colcon build --symlink-install
```

### Setup Workspace Environment
Add to your `~/.bashrc`:

```bash
# ROS 2 Workspace Setup
source ~/ros2_ws/install/setup.bash
```

## A.7 Common Troubleshooting

### ROS 2 Network Issues
**Problem**: Nodes on different machines cannot communicate.

**Solution**:
1. Ensure all machines are on the same network
2. Set ROS_DOMAIN_ID consistently across machines:
   ```bash
   export ROS_DOMAIN_ID=0
   ```
3. Check firewall settings to allow DDS communication

### Gazebo Performance Issues
**Problem**: Gazebo runs slowly or crashes.

**Solutions**:
1. Ensure proper graphics drivers are installed
2. Reduce simulation complexity (fewer objects, simpler models)
3. Adjust Gazebo settings in the world file:
   ```xml
   <real_time_update_rate>1000</real_time_update_rate>
   <max_step_size>0.001</max_step_size>
   ```

### Isaac Sim Installation Issues
**Problem**: Isaac Sim fails to launch.

**Solutions**:
1. Verify NVIDIA GPU and drivers:
   ```bash
   nvidia-smi
   ```
2. Check OpenGL support:
   ```bash
   glxinfo | grep "OpenGL renderer"
   ```
3. Ensure sufficient VRAM (minimum 8GB recommended)

### Python Package Conflicts
**Problem**: Import errors or version conflicts.

**Solution**:
1. Use virtual environments consistently
2. Check package versions:
   ```bash
   pip list | grep <package_name>
   ```
3. Reinstall packages in a clean virtual environment

### Camera Sensor Issues
**Problem**: Camera topics show no data.

**Solutions**:
1. Check camera permissions:
   ```bash
   sudo usermod -a -G video $USER
   ```
2. Verify camera is detected:
   ```bash
   ls /dev/video*
   ```
3. Check Gazebo camera plugin configuration

## A.8 Development Environment Setup

### VS Code for ROS 2 Development
Install the ROS extension for VS Code:
1. Install VS Code
2. Install the "ROS" extension by Microsoft
3. Install the "C/C++" extension
4. Configure for ROS 2 Humble

### Useful VS Code Settings
Add to your VS Code `settings.json`:
```json
{
    "python.defaultInterpreterPath": "~/robotics_env/bin/python",
    "terminal.integrated.env.linux": {
        "ROS_DISTRO": "humble",
        "ROS_VERSION": "2"
    }
}
```

## A.9 Simulation Environment Setup

### Create a Basic Robot Model
1. Create URDF files for your robot in `~/ros2_ws/src/robot_description/urdf/`
2. Create launch files in `~/ros2_ws/src/robot_description/launch/`
3. Build the workspace:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select robot_description
   ```

### Test Robot in Gazebo
```bash
# Source your workspace
source ~/ros2_ws/install/setup.bash

# Launch your robot in Gazebo
ros2 launch robot_description spawn_robot.launch.py
```

## A.10 Unitree Go2/G1 Setup (If Available)

### Network Configuration
1. Connect to robot's WiFi or Ethernet
2. Ensure proper IP addressing
3. Test connectivity:
   ```bash
   ping <robot_ip_address>
   ```

### ROS 2 Interface
1. Install Unitree ROS 2 packages
2. Source the robot's ROS environment
3. Test basic commands:
   ```bash
   ros2 topic list
   ros2 service list
   ```

## A.11 Performance Optimization Tips

### Simulation Performance
- Reduce physics update rate if not needed
- Simplify collision meshes for dynamic objects
- Use fewer but more efficient sensors
- Consider using CPU-based physics if GPU is overloaded

### Real-time Performance
- Use real-time kernel if available
- Set appropriate process priorities
- Minimize unnecessary computation in control loops
- Use efficient data structures and algorithms

## A.12 Common Error Messages and Solutions

### "Command 'ros2' not found"
**Cause**: ROS 2 environment not sourced.
**Solution**:
```bash
source /opt/ros/humble/setup.bash
```

### "ModuleNotFoundError: No module named 'rclpy'"
**Cause**: Python environment not configured properly.
**Solution**:
```bash
source /opt/ros/humble/setup.bash
source ~/robotics_env/bin/activate
```

### "Unable to register node: Node name already exists"
**Cause**: Multiple instances of the same node running.
**Solution**: Kill existing processes or use unique node names.

### "Could not find a package configuration file"
**Cause**: Package not built or not in workspace.
**Solution**:
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## A.13 Backup and Recovery

### Workspace Backup
```bash
# Create a backup of your workspace
tar -czf ~/robotics_workspace_backup_$(date +%Y%m%d).tar.gz ~/ros2_ws
```

### System Restore Points
Consider using system restore tools like Timeshift for Ubuntu:
```bash
sudo apt install timeshift
```

## A.14 Additional Resources

### Official Documentation
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Gazebo Documentation](https://gazebosim.org/docs)
- [NVIDIA Isaac Documentation](https://docs.nvidia.com/isaac/)
- [Unitree Robotics Documentation](https://www.unitree.com/docs/)

### Community Support
- [ROS Answers](https://answers.ros.org/questions/)
- [Gazebo Answers](https://answers.gazebosim.org/)
- [Robotics Stack Exchange](https://robotics.stackexchange.com/)

### Troubleshooting Tools
- `ros2 doctor`: Check ROS 2 installation health
- `ros2 run rqt_graph rqt_graph`: Visualize node connections
- `ros2 topic echo <topic_name>`: Monitor topic data

This appendix will continue to be updated with additional troubleshooting tips and setup guides as needed. If you encounter issues not covered here, please consult the official documentation for the specific tools or reach out to the robotics community for assistance.
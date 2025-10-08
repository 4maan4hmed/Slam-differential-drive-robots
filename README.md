# my_bot

This repository contains the ROS 2 package **my_bot**. It provides the necessary files and instructions to simulate, control, and perform SLAM (Simultaneous Localization and Mapping) with a custom robot using ROS 2.

## Features

- Robot description (URDF/Xacro)
- Simulation support (Gazebo)
- Basic control nodes
- SLAM implementation
- Launch files for easy startup

## Installation

Clone the repository into your ROS 2 workspace:

```bash
cd ~/ros2_ws2/src
git clone <repository-url>
cd ~/ros2_ws2
colcon build
source install/setup.bash
```

## Usage

To launch the robot simulation:

```bash
ros2 launch my_bot my_bot.launch.py
```

To run SLAM:

```bash
ros2 launch my_bot slam.launch.py
```

## Directory Structure

- `urdf/` - Robot description files
- `launch/` - Launch files
- `config/` - Configuration files
- `src/` - Source code for nodes

## Requirements

- ROS 2 (Humble or newer)
- Gazebo (for simulation)
- SLAM package (e.g., slam_toolbox or gmapping)

## License

This project is licensed under the MIT License.

## Author

Amaan


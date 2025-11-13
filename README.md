# TEL2025_NCHU

This repository contains the code and resources for the TEL2025 project at National Chung Hsing University (NCHU). The project focuses on [brief description of the project].

---
## ros2_ws
This directory contains the ROS 2 workspace for the TEL2025 project. It includes various ROS 2 packages and nodes developed for the project.

Download the workspace:
```bash
git clone https://gitlab.com/tel2025_nchu/ros2_ws.git
```

The Dependencies:
```bash
# serial
sudo apt install ros-humble-serial
# joy
sudo apt install ros-humble-joy
```

Build the workspace:
```bash
cd ~/ros2_ws
colcon build
```
if you want to build with scripts linked to the source files, use:
```bash
colcon build --symlink-install
```

Source the workspace:
```
source install/setup.bash
```

---
### AT9S
This directory contains the code and resources for interfacing with accessory AT9S controller data from Arduino to ROS2.
The node publishes the controller data to the `/joy` topic.

### Directly run the node:
```bash
ros2 run at9s at9s_node
```
or you can use:
```bash
ros2 launch at9s at9s_launch.py port:=<your_serial_port>
```

### Notes:
1. The defualt serial port is `/dev/ttyACM1`. You can change it by modifying the `port` parameter in the `at9s_node` executable.
2. The baudrate is set to `115200`. You cannot change it in the current version.
3. If package not found, run: `cd ~/ros2_ws && source install/setup.bash`
---
### robot_communicate
This directory contains the code and resources for communicating with the robot via serial communication.

### Directly run the node:
```bash
ros2 run robot_communicate robot_communicate_node
```
or you can use:
```bash
ros2 launch robot_communicate robot_communicate_launch.py port:=<your_serial_port>
```
### Notes:
1. The defualt serial port is `/dev/ttyACM0`. You can change it by modifying the `port` parameter in the `robot_communicate_node` executable.
2. The baudrate is set to `115200`. You cannot change it in the current version.
3. If package not found, run: `cd ~/ros2_ws && source install/setup.bash`
---
### RealSense Camera
To be added in the future.

---
### Object Detection
To be added in the future.

---
## Arduino
This directory contains the Arduino code for interfacing with the AT9S controller, baseball robot control code,and communicating with the ROS2 node.

### AT9S
The `AT9S` folder contains the Arduino code for reading data from the AT9S controller and sending it to the ROS2 node via serial communication.

Dependencies of AT9S:
```bash
# SBUS
git clone https://github.com/batteryouo/SBUS.git
```
### Robot Control
The `robot_control` folder contains the Arduino code for controlling the baseball robot based on commands received from the ROS2 node via serial communication.
```bash
# MPU6050
git clone https://github.com/batteryouo/mpu6050.git
```
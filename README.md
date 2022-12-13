# Hokuyo-Data-Reading
This document is mainly about how to retrieve data from the Hokuyo UST-20LX laser connected to the RPi 3, and visualize the cloud points on your own desktop or any device.

## Prerequisites

+ OS: Ubuntu 20.04 LTS
+ ROS 2: Galactic
+ Create a ROS 2 workspace
+ Download the package: [urg_node2](https://github.com/Hokuyo-aut/urg_node2)
```
$ cd <ROS2_workspace>/src
$ git clone --recursive https://github.com/Hokuyo-aut/urg_node2.git
```

## Raspberry Pi 3
1. Broadcast the laser's ethernet IP address to RPi 3
```
$ sudo ip addr add 192.168.0.15/24 broadcast 192.168.1.255 dev eth0
```
2. Under Ros 2 worksapce, run `rosdep` to check for missing dependencies before building
```
$ rosdep install -i --from-path src --rosdistro galactic -y
```
3. Build the package
```
$ colcon build --packages-select urg_node2
```
4. Source the setup files
```
$ . install/setup.bash
```
5. Launch the node to active state to scan the distribution of the cloud points
```
$ ros2 launch urg_node2 urg_node2.launch.py
```

## Desktop
1. Under ROS 2 workspace, build the package
```
$ colcon build --packages-select urg_node2
```
2. Source the setup files
```
$ . install/setup.bash
```
3. Activate the node and collect the data from RPi 3
```
$ ros2 run urg_node2 reading_laserscan 
```
4. Click the enter button, the lidar will scan once and save into the CSV file 


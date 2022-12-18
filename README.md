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
$ colcon build --packages-select laserscan_subscriber
```
2. Source the setup files
```
$ . install/setup.bash
```
3. Activate the node and collect the data from RPi 3
```
$ ros2 run laserscan_subscriber laserscan_subscriber 
```
4. Click the enter button, the lidar will scan once and save the data into the CSV file, including the position of objects in the body frame, the distance data, and the corresponding angle data.

## Code explannation
The [laserscan_subscriber](https://github.com/davidwater/hokuyo-datareading-urg-node2/blob/main/laserscan_subscriber/src/laserscan_subscriber.cpp) was created for subscribing the data from the laser. We used the [Eigen library](https://eigen.tuxfamily.org/index.php?title=Main_Page) to build the data structure. The data are saved as CSV file, the first col. is distance data, the second is angle data, the third and the fourth are calculated by the easy trigonometric function to put the (x,y) location.
Function `topic_callback` specifys the way we adopt.
```
void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        cout<<"IN CALLBACK!!!"<<endl;
        //int anmin = 440;
        //int anmax = 640;
        int rm = 0; // to remove on each side of scan (0.25deg per measure)
        int anmin = rm;
        int anmax = 1080-rm;
        double div = 1.0; // multiply env for norm
        // reset data
        int j=0; // idx in data
        data.conservativeResize(j,NoChange);
        for (int i = anmin; i < anmax; i++) {
            dist[i] = msg->ranges[i];
            //if(dist[i]<=7 && dist[i]>=1){ // aritifcially limiting range
                data.conservativeResize(j+1,NoChange); // resize mat
                data(j,2) = dist[i]*div;  // set dist
                data(j,3) = theta(i);
                data(j,0) = data(j,2) * cos(theta(i)); // x
                data(j,1) = data(j,2) * sin(theta(i)); // y
                j++; // iterate
            //}
        }
        Scan scan;
        scan.data = data;
        scan.loc =  Vector2d({0, 0});
        scan.ori = 0;
        scan.write_scan(n++);
        cout << "*********************************************************" << endl;
        cin.get();
    }
```

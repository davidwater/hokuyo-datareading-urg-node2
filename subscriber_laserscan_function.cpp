// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <functional>
#include <memory>
#include <eigen3/Eigen/Dense>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;
using Eigen::VectorXd;
using Eigen::MatrixXd;

 

class ReadingLaserscan : public rclcpp::Node
{
public:
  VectorXd dist = VectorXd(1080);
  VectorXd theta = VectorXd::LinSpaced(1080,0, 1.5*M_PI); // 0 ~ 1.5*pi rad
  MatrixXd data = MatrixXd::Zero(1080,2);

public:
  ReadingLaserscan()
  : Node("reading_laserscan")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10, std::bind(&ReadingLaserscan::topic_callback, this, _1));
  }



private:
  void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    for (int i=0; i < 1080; i++)
    {
      dist[i] = msg->ranges[i];
    }
    data.col(0)=dist;
    data.col(1)=theta;
    std::cout << "     Latest data: \n" << "distance (m) angle (rad) \n"<< data << std::endl;
  }
  
public: 
  MatrixXd get_data()
  {
    return data;
  }

rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;

};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ReadingLaserscan>());
  rclcpp::shutdown();
  ReadingLaserscan reader;
  MatrixXd data_ = reader.get_data();

  return 0;
}

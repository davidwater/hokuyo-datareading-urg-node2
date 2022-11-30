#include <functional>
#include <memory>
#include <eigen3/Eigen/Dense>

#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

using std::placeholders::_1;
using namespace std;
using Eigen::VectorXf;
using Eigen::MatrixXd;
using Eigen::MatrixXf;

class ReadingPoint : public rclcpp::Node
{
public:
    ReadingPoint()
    : Node("reading_point")
    {
      auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
      subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", sensor_qos,std::bind(&ReadingPoint::point_callback, this, _1));
    }
    

private:
    void point_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        //cout << "in callback!" << endl;
        position(0) = msg->pose.pose.position.x;
        position(1) = msg->pose.pose.position.y;
        cout << "Position: " << position << endl;
        //RCLCPP_INFO(this->get_logger(), "Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    }
    MatrixXf position = MatrixXf(1,2); // (x, y) location 
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};


int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ReadingPoint>());
  rclcpp::shutdown();

  return 0;
}

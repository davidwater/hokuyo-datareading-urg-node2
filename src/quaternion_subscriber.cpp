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
using Eigen::VectorXd;
using Eigen::MatrixXd;

class ReadingQuaternion : public rclcpp::Node
{
public:
    ReadingQuaternion()
    : Node("reading_quaternion")
    {
      auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
      subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", sensor_qos, std::bind(&ReadingQuaternion::topic_callback, this, _1));
    }

private:
    void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {        
        double x, y, z, w, yaw;
        x = msg->pose.pose.orientation.x;
        y = msg->pose.pose.orientation.y;
        z = msg->pose.pose.orientation.z;
        w = msg->pose.pose.orientation.w;
        yaw = atan2 (2*(w*z+x*y), 1-2*(pow(y,2) + pow(z,2)));
        theta = yaw;
        cout << "Yaw angle (degree): " << theta << endl;
    }
    double theta; // yaw angle
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};


int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ReadingQuaternion>());
  rclcpp::shutdown();

  return 0;
}

#include <functional>
#include <memory>
#include <eigen3/Eigen/Dense>

#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;
using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;

class ReadingLaserscan : public rclcpp::Node
{
public:
    ReadingLaserscan()
    : Node("reading_laserscan")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 1000, std::bind(&ReadingLaserscan::topic_callback, this, _1));
    }
    MatrixXd get_data()
    {
        return data;
    }
    bool isEmpty(){
        return empty;
    }
private:
    void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        for (int i=0; i < 1080; i++)
        {
            dist[i] = msg->ranges[i];
        }
        data.col(2)=dist;
        data.col(3)=theta;
        data.col(0)= dist.array() * theta.array().cos();
        data.col(1)= dist.array() * theta.array().sin();
    }
    bool empty =1;
    VectorXd dist = VectorXd(1080);
    VectorXd theta = VectorXd::LinSpaced(1080,0, 1.5*M_PI); // 0 ~ 1.5*pi rad
    MatrixXd data = MatrixXd::Zero(1080,4);
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};



int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ReadingLaserscan>());
  rclcpp::shutdown();
  //ReadingLaserscan reader;
  //while(reader.isEmpty());
  //cout<<"getting data!"<<endl;
  //MatrixXd data_ = reader.get_data();
  //cout<<data_<<endl;

  return 0;
}

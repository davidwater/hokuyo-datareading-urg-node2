#include <functional>
#include <memory>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <curses.h>
#include <cmath>
#include <fstream>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;
using namespace std;
using namespace Eigen;

static int n=0;

class Scan {
  // Matrix<double, Dynamic, 2> pts, mes;
public:
  Matrix<double, Dynamic, 4> data; // x, y, d, an
  Vector2d loc;
  double ori;
  void write_scan(int n) {
    ofstream f("scan_" + to_string(n) + ".csv");
    // first line: posx,posy,ori,rows
    //f << loc(0) << "," << loc(1) << "," << ori << "," << data.rows() << endl;
    f <<"," << data.rows() << endl;
    // subsequent lines: x,y,d,an
    for (int i = 0; i < data.rows(); i++)
      f << data(i, 0) << "," << data(i, 1) << "," << data(i, 2) << ","
        << data(i, 3) << endl;
    f.close();
  }
};




class LaserScanReader : public rclcpp::Node {
public:
    LaserScanReader()
            : Node("laser_scan_reader") {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan", 10, std::bind(&LaserScanReader::topic_callback, this, _1));
    }

    MatrixXd get_data() {
        return data;
    }

private:
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

    VectorXd dist = VectorXd(1080);
    VectorXd theta = VectorXd::LinSpaced(1080, -0.75* M_PI,  0.75* M_PI); // 0 ~ 1.5*pi rad
    MatrixXd data = MatrixXd::Zero(1080, 4);
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};


int main(int argc, char *argv[]) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserScanReader>());
    rclcpp::shutdown();

    return 0;
}

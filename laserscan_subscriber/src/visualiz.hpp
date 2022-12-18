#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP

#include "plotty/matplotlibcpp.hpp"
#include <Eigen/Core>

using namespace Eigen;
using namespace std;
namespace plt = plotty;

class Visualizer {
public:
    Visualizer();

    void add_points(Eigen::Matrix<double, Eigen::Dynamic, 2> pts,
                    const char *label);

    void add_ellipse(Eigen::VectorXd p, const char *label);

    void add_line(Eigen::VectorXd p, const char *label);

    void show();
};

Visualizer::Visualizer() {
    plt::figure();
    plt::axis("equal");
    plt::ylim(-10, 10);
    plt::xlim(-20, 0);
}

void Visualizer::add_ellipse(Eigen::VectorXd p, const char *label) {
    Eigen::ArrayXd t = Eigen::ArrayXd::LinSpaced(100, 0, 2 * M_PI);
    Eigen::ArrayXd X = t.cos().abs().pow(p[5]) * p[3] * t.cos().sign();
    Eigen::ArrayXd Y = t.sin().abs().pow(p[5]) * p[4] * t.sin().sign();
    Eigen::MatrixXd rotm(2, 2);
    rotm << cos(p[2]), -sin(p[2]), sin(p[2]), cos(p[2]);
    Eigen::MatrixXd d(X.size(), 2);
    d.col(0) = X.matrix();
    d.col(1) = Y.matrix();
    Eigen::MatrixXd rotd = (rotm * d.transpose()).transpose();
    rotd.col(0) = rotd.col(0).array() + p[0];
    rotd.col(1) = rotd.col(1).array() + p[1];
    plt::plot(rotd.col(0), rotd.col(1), label);
}

void Visualizer::add_line(Eigen::VectorXd p, const char *label) {

    Eigen::Matrix<double, 2, 2> pts;
    // find center point
    double c_x = p(0) * cos(p(1));
    double c_y = p(0) * sin(p(1));
    //Visualizer::add_points(Eigen::Matrix<double, 1, 2>({c_x, c_y}), "r.");
    // y=ax+b perpen
    double a = -c_x / c_y;
    double b = c_y + c_x * c_x / c_y;
    // points to draw
    double z_x1 = 100;
    double z_y1 = a * z_x1 + b;
    double z_x2 = -100;
    double z_y2 = a * z_x2 + b;
    pts << z_x1, z_y1, z_x2, z_y2;
    Visualizer::add_points(pts, label);
}


void Visualizer::add_points(Eigen::Matrix<double, Eigen::Dynamic, 2> pts,
                            const char *label) {
    plt::plot(pts(Eigen::all, 0), pts(Eigen::all, 1), label);
}

void Visualizer::show() { plt::show(); }

#endif // VISUALIZER_HPP

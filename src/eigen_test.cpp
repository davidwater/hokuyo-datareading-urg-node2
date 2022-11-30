#include <iostream>
#include <eigen3/Eigen/Dense>
 
using Eigen::MatrixXd;
using Eigen::VectorXd;
 
int main()
{
  
  MatrixXd data = MatrixXd (1080,2);
  std::cout << data << std::endl;
  MatrixXd m = MatrixXd::Random(3,3);
  std::cout << m << std::endl;
  std::cout << MatrixXd::Constant(3,3,1.2) << std::endl;
  m = (m + MatrixXd::Constant(3,3,1.2)) * 50;
  //std::cout << "m =" << std::endl << m << std::endl;
  VectorXd v = VectorXd::LinSpaced(1080, 0, 1.5*M_PI);
  //std::cout << "v= \n" << v << std::endl;
  std::cout << v[0] << std::endl ;
  //std::cout << "m * v =" << std::endl << m * v << std::endl;
}


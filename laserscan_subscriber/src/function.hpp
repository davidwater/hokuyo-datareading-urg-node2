#ifndef FUNCTIONS_HPP
#define FUNCTIONS_HPP
#include "visualizer.hpp"
#include <Eigen/LU>
#include <autodiff/reverse/var.hpp>
#include <ceres/autodiff_cost_function.h>
#include <ceres/cost_function.h>
#include <ceres/problem.h>
#include <ceres/solver.h>
static Visualizer v;
// simple functions
struct Scan {
    // Matrix<double, Dynamic, 2> pts, mes;
    Matrix<double, Dynamic, 4> data; // x, y, d, an
    Vector<double, 2> loc;
    double ori;
};
// sign
template <typename T> int sgn(T val) { return (T(0) < val) - (val < T(0)); }

double cov(VectorXd x, VectorXd y) {
  auto xm = x.array() - x.mean();
  auto ym = y.array() - y.mean();
  auto m = xm.array() * ym.array();
  return m.sum() / (m.size() - 1);
}
VectorXd atan2(VectorXd y, VectorXd x) {
  VectorXd res(y.size());
  for (int i = 0; i < y.size(); i++) {
    res(i) = atan2(y(i), x(i));
  }
  return res;
}
namespace ad = autodiff;
// MODEL 0: ELLIPSES
template <typename T>
T ft_0(T xc, T yc, T th, T a, T b, T e, T xp, T yp, T d, T an, T mud, T muan) {
  T x = xp + (d + mud) * cos(an + muan);
  T y = yp + (d + mud) * sin(an + muan);
  T f1 = ((x - xc) * cos(th) + (y - yc) * sin(th)) / a;
  T f2 = ((x - xc) * sin(th) - (y - yc) * cos(th)) / b;
  T buf = pow(pow(f1, 2.), 1. / e) + pow(pow(f2, 2.), 1. / e);
  return pow(buf, e) - 1.;
}
template <typename T>
T fs_0(const VectorXd p, const VectorXd pos, const VectorXd data) {
  // return ft<T, T, T, T, T, T, T, T, T, T, T, T>(xc, yc, th, a, b, e, xp, yp,
  // d,
  //                                              an, 0, 0);
  return ft_0<T>(p(0), p(1), p(2), p(3), p(4), p(5), pos(0), pos(1), data(2),
                 data(3), 0, 0);
}

/*! \fn VectorXd fsn_0(const Vector<double, 6> &p, const Vector2d &pos, const
   Matrix<double, Dynamic, 2> &m) \
    \brief Implicit ellipse function
    \param p function parameters
    \param pos position of scanner
    \param m matrix of measurements
    \return returns error
*/
VectorXd fsn_0(const VectorXd p, const VectorXd pos, const MatrixXd data) {
  VectorXd ans(data.rows());
  for (int i = 0; i < data.rows(); i++)
    ans(i) = fs_0<double>(p, pos, data(i, all));
  return ans;
}

/*! \fn VectorXd dfsn_0(VectorXd p, VectorXd pos, MatrixXd m)
    \brief Implicit ellipse derivated function
    \param p function parameters
    \param pos position of scanner
    \param m matrix of measurements
    \return returns implicit derivative
*/
MatrixXd dfsn_0(const VectorXd p, const VectorXd pos, const MatrixXd data) {
  ad::var xc(p(0)), yc(p(1)), th(p(2)), a(p(3)), b(p(4)), e(p(5)), xp(pos(0)),
      yp(pos(1)), d(0.), an(0.), mud(0.), muan(0.);
  ad::var f = ft_0(xc, yc, th, a, b, e, xp, yp, d, an, mud, muan);

  MatrixXd ans(data.rows(), 12);
  for (unsigned i = 0; i < data.rows(); i++) {
    d.update(data.row(i)(2));
    an.update(data.row(i)(3));
    f.update();
    auto [dxc, dyc, dth, da, db, de, dxp, dyp, dd, dan, dmud, dmuan] =
        ad::derivatives(f,
                        ad::wrt(xc, yc, th, a, b, e, xp, yp, d, an, mud, muan));
    ans(i, 0) = dxc;
    ans(i, 1) = dyc;
    ans(i, 2) = dth;
    ans(i, 3) = da;
    ans(i, 4) = db;
    ans(i, 5) = de;
    ans(i, 6) = dxp;
    ans(i, 7) = dyp;
    ans(i, 8) = dd;
    ans(i, 9) = dan;
    ans(i, 10) = dmud;
    ans(i, 11) = dmuan;
  }
  return ans;
}

MatrixXd dop_0(const VectorXd p, const VectorXd pos, const MatrixXd data,
               const double sigma_d) {
  Matrix<double, Dynamic, 12> df = dfsn_0(p, pos, data);
  MatrixXd Jmes = df(all, {10, 11});
  MatrixXd Emes = Jmes * sigma_d * sigma_d * Jmes.transpose();
  MatrixXd H = df(all, {0, 1, 2, 3, 4, 5});
  MatrixXd En = (H.transpose() *
                 Emes.completeOrthogonalDecomposition().pseudoInverse() * H)
                    .completeOrthogonalDecomposition()
                    .pseudoInverse();
  return En;
}

VectorXd init_0(VectorXd x, VectorXd y, VectorXd loc) {
  double *p = new double[9];
  MatrixXd m({{cov(x, x), cov(x, y)}, {cov(y, x), cov(y, y)}});
  JacobiSVD<MatrixXd> svd(m, ComputeFullU | ComputeFullV);
  auto V = svd.matrixV();
  auto D = svd.singularValues();
  double cond = D(0) / D(D.size() - 1);
  double th0 = atan2(V(1, 0), V(0, 0));
  // rotation of ptcloud
  MatrixXd d(x.size(), 2);
  d.col(0) = x;
  d.col(1) = y;
  MatrixXd rotm(2, 2);
  rotm << cos(-th0), -sin(-th0), sin(-th0), cos(-th0);
  MatrixXd rotd = (rotm * d.transpose()).transpose();
  double x0 = d.col(0).mean();
  double y0 = d.col(1).mean();
  double a0 = (rotd.col(0).maxCoeff() - rotd.col(0).minCoeff()) / 2;
  double b0 = (rotd.col(1).maxCoeff() - rotd.col(1).minCoeff()) / 2;
  double angle = atan2(y0 - loc(1), x0 - loc(0));
  p[0] = x0 + sqrt(a0 + b0) * cos(angle); // pushes too much !!!
  p[1] = y0 + sqrt(a0 + b0) * sin(angle);
  p[6] = x0 + sqrt(a0 * a0 + b0 * b0) * cos(angle); // alt x0 and y0
  p[7] = y0 + sqrt(a0 * a0 + b0 * b0) * sin(angle);
  // using old a0 b0
  a0 = sqrt(2 * D(0)); // a0
  b0 = sqrt(2 * D(1)); // b0
  //
  if (b0 < 1)
    b0 = 1;
  if (a0 < 1)
    a0 = 1;
  p[2] = th0;
  p[3] = a0;
  p[4] = b0;
  p[5] = 0.1;            // CHANGED
  p[8] = th0 + M_PI / 2; // alt th0
  VectorXd p_i = Map<Vector<double, 9>>(p);
  return p_i;
}

struct LossFunction_0 {
  LossFunction_0(double x, double y) : _x(x), _y(y) {}
  template <typename T> bool operator()(const T *const p, T *residual) const {
    T f1 = ((_x - p[0]) * cos(p[2]) + (_y - p[1]) * sin(p[2])) / p[3];
    T f2 = ((_x - p[0]) * sin(p[2]) - (_y - p[1]) * cos(p[2])) / p[4];
    residual[0] =
        p[4] * p[3] * // penalty on area
        // exp(abs(p[5] - 1.0)) * // penalise eps, pow 1 = no penalty
        (pow(pow(pow(f1, 2.0), (1.0 / p[5])) + pow(pow(f2, 2.0), (1.0 / p[5])),
             p[5]) -
         1.0);
    return true;
  }

private:
  double _x;
  double _y;
};

VectorXd ls_0(const VectorXd loc, const MatrixXd data) {
  VectorXd xdata = data(all, 0);
  VectorXd ydata = data(all, 1);
  auto p0 = init_0(xdata, ydata, loc);
  double p0arr[6] = {p0[0], p0[1], p0[2], p0[3], p0[4], p0[5]};
  double p1[6] = {p0[0], p0[1], p0[2], p0[3], p0[4], 1.9}; // ch
  // .eps
  //   viz
  //  v.add_object({{p0[0], p0[1], p0[2], p0[3], p0[4], p0[5]}, 0}, "b-");
  //
  double p2[6] = {p0[6], p0[7], p0[2], p0[3], p0[4], 0.1}; // CHANGED
  double p3[6] = {p0[6], p0[7], p0[2], p0[3], p0[4], 1.9}; // ch.
                                                           // eps
  //   theta variation
  double p4[6] = {p0[0], p0[1], p0[8], p0[3], p0[4], p0[5]};
  double p5[6] = {p1[0], p1[1], p1[8], p1[3], p1[4], p1[5]};
  double p6[6] = {p2[0], p2[1], p2[8], p2[3], p2[4], p2[5]};
  double p7[6] = {p3[0], p3[1], p3[8], p3[3], p3[4], p3[5]};
  const int n = 8;
  double *pa[n] = {p0arr, p1, p2, p3, p4, p5, p6, p7};
  // double *pa[] = {p0.data(), p1, p2, p3};
  // double *pa[] = {p2, p3};
  // ceres::Problem *problem = new ceres::Problem[n]; // destructor segfaults
  ceres::Problem problem[n];
  for (unsigned i = 0; i < xdata.size(); i++)
    for (unsigned k = 0; k < n; k++) {
      ceres::CostFunction *cost_function =
          new ceres::AutoDiffCostFunction<LossFunction_0, 1, 6>(
              new LossFunction_0(xdata(i), ydata(i)));
      problem[k].AddResidualBlock(cost_function, nullptr, pa[k]);
    }
  // bounds
  for (unsigned i = 0; i < n; i++) {
    problem[i].SetParameterLowerBound(pa[i], 3, 1);    // a
    problem[i].SetParameterLowerBound(pa[i], 4, 1);    // b
    problem[i].SetParameterLowerBound(pa[i], 5, 0.01); // eps <- very important
    problem[i].SetParameterUpperBound(pa[i], 3, 30);   // a
    problem[i].SetParameterUpperBound(pa[i], 4, 30);   // b
    problem[i].SetParameterUpperBound(pa[i], 5, 1.99); // eps
    // avoids auto-occlusion
    problem[i].SetParameterUpperBound(pa[i], 0, max(p0[6], p0[0])); // x
    problem[i].SetParameterLowerBound(pa[i], 0, min(p0[6], p0[0])); // x
    problem[i].SetParameterUpperBound(pa[i], 1, max(p0[7], p0[1])); // y
    problem[i].SetParameterLowerBound(pa[i], 1, min(p0[7], p0[1])); // y
  }
  ceres::Solver::Options options;
  options.num_threads = 24;
  options.minimizer_progress_to_stdout = false;
  options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY; //<- slower
  options.preconditioner_type = ceres::CLUSTER_JACOBI;       //<- also slower
  options.initial_trust_region_radius = 1e8;                 // important
  options.max_num_iterations = 100;
  ceres::Solver::Summary summary[n];
  for (unsigned i = 0; i < n; i++)
    ceres::Solve(options, &problem[i], &summary[i]);
  unsigned opt = 0;
  double min_cost = 10000;

  for (unsigned i = 0; i < n; i++) {
    cout << pa[i][0] << ", " << pa[i][1] << ", " << pa[i][2] << ", " << pa[i][3]
         << ", " << pa[i][4] << ", " << pa[i][5] << endl;
    // checking that array is finite (not nan or inf)
    bool arrayIsFinite = 1;
    for (int k = 0; k < 6; k++)
      if (!isfinite(pa[i][k])) {
        arrayIsFinite = 0;
        break;
      }
    if (!arrayIsFinite)
      continue;
    // if not skipped in continue, has a chance at being optimal sol
    if (summary[i].final_cost < min_cost && arrayIsFinite) {
      min_cost = summary[i].final_cost;
      opt = i;
    }
  }
  Vector<double, 6> ans = Map<Vector<double, 6>>(pa[opt]);
  cout << "LS YIELDS: " << ans.transpose() << endl;
  return ans;
}

bool safety_0(VectorXd &p) {
  if (p(5) < 0.1) {
    p(5) = 0.1;
    return 1;
  }
  if (p(5) > 1.9) {
    p(5) = 1.9;
    return 1;
  }
  p(2) = fmod(p(2),2*M_PI);
  if(p(2)<0)
    p(2)+=2*M_PI;
  return 0;
}

// MODEL 1: LINES (not passing through origin)
template <typename T> // (p.71) p_seg][r,a]
T ft_1(T r, T a, T xp, T yp, T d, T an, T mud, T muan) {
  T x = xp + (d + mud) * cos(an + muan);
  T y = yp + (d + mud) * sin(an + muan);
  T testing_angle = atan2(y, x);
  /*if (testing_angle >= min(th_min, th_max) &&
      testing_angle <= max(th_min, th_max)) {
    return (x * cos(a) + y * sin(a)) / r - 1.;
  } else
    return 0.;*/
  return ((x * cos(a) + y * sin(a)) / r) - 1.;
}
template <typename T>
T fs_1(const VectorXd p, const VectorXd pos, const VectorXd data) {
  // return ft<T, T, T, T, T, T, T, T, T, T, T, T>(xc, yc, th, a, b, e, xp, yp,
  // d,
  //                                              an, 0, 0);
  return ft_1<T>(p(0), p(1), pos(0), pos(1), data(2), data(3), 0, 0);
}

/*! \fn VectorXd fsn_0(const Vector<double, 6> &p, const Vector2d &pos, const
   Matrix<double, Dynamic, 2> &m) \
    \brief Implicit ellipse function
    \param p function parameters
    \param pos position of scanner
    \param m matrix of measurements
    \return returns error
*/
VectorXd fsn_1(const VectorXd p, const VectorXd pos, const MatrixXd data) {
  VectorXd ans(data.rows());
  for (int i = 0; i < data.rows(); i++)
    ans(i) = fs_1<double>(p, pos, data(i, all));
  return ans;
}

/*! \fn VectorXd dfsn_0(VectorXd p, VectorXd pos, MatrixXd m)
    \brief Implicit ellipse derivated function
    \param p function parameters
    \param pos position of scanner
    \param m matrix of measurements
    \return returns implicit derivative
*/
MatrixXd dfsn_1(const VectorXd p, const VectorXd pos, const MatrixXd data) {
  ad::var r(p(0)), a(p(1)), xp(pos(0)), yp(pos(1)), d(0.), an(0.), mud(0.),
      muan(0.);
  ad::var f = ft_1(r, a, xp, yp, d, an, mud, muan);

  MatrixXd ans(data.rows(), 8);
  for (unsigned i = 0; i < data.rows(); i++) {
    d.update(data.row(i)(2));
    an.update(data.row(i)(3));
    f.update();
    auto [dr, da, dxp, dyp, dd, dan, dmud, dmuan] =
        ad::derivatives(f, ad::wrt(r, a, xp, yp, d, an, mud, muan));
    ans(i, 0) = dr;
    ans(i, 1) = da;
    ans(i, 2) = dxp;
    ans(i, 3) = dyp;
    ans(i, 4) = dd;
    ans(i, 5) = dan;
    ans(i, 6) = dmud;
    ans(i, 7) = dmuan;
  }
  return ans;
}

MatrixXd dop_1(const VectorXd p, const VectorXd pos, const MatrixXd data,
               const double sigma_d) {
  Matrix<double, Dynamic, 8> df = dfsn_1(p, pos, data);
  MatrixXd Jmes = df(all, {6, 7});
  MatrixXd Emes = Jmes * sigma_d * sigma_d * Jmes.transpose();
  MatrixXd H = df(all, {0, 1});
  MatrixXd En = (H.transpose() *
                 Emes.completeOrthogonalDecomposition().pseudoInverse() * H)
                    .completeOrthogonalDecomposition()
                    .pseudoInverse();
  return En;
}

VectorXd ls_1(const VectorXd loc, const MatrixXd data) {
  Vector<double, 2> p;
  MatrixXd X = data(all, {0, 1});
  MatrixXd b = (X.transpose() * X).inverse() * X.transpose() *
               Matrix<double, Dynamic, 1>::Ones(X.rows(), 1);
  double r = sgn(b(0)) / sqrt(b(0) * b(0) + b(1) * b(1));
  double a = atan2(b(1) * r, b(0) * r);
  VectorXd angles = atan2(X.col(1), X.col(0));
  p << r, a; //, angles.minCoeff(), angles.maxCoeff();
  // cout << p.transpose() << endl;
  return p;
}
bool safety_1(VectorXd &p) { return 0; }
#endif // FUNCTIONS_HPP

#ifndef EKF_HPP
#define EKF_HPP
#include "model.hpp"
class Ekf {
public:
  Ekf(const Model *m, VectorXd p, MatrixXd E) : _model(m), _p(p), _E(E) {}
  int update(const VectorXd data, const VectorXd pose, const double tol) {
    cout << "  in ekf" << endl;
    // system propagation
    _E = _E + _model->_Q_s;
    // cout << "   E:" << _E << endl;
    cout << "   p:" << _p.transpose() << endl;
    if (_model->_safety(_p))
      cout << "forced safe values on p" << endl;
    _m = data;
    // cout << "   r" << endl;
    _r = -_model->_fs(_p, pose, _m)(0);
    // cout << "   df" << endl;
    MatrixXd df = _model->_dfs(_p, pose, _m);
    H = df(all, _model->_dfs_parameter_indexes);
    // cout << "   H:" << H << endl;
    J = df(all, _model->_dfs_measurement_error_indexes); // see dfsn_0
    // cout << "   J:" << J << endl;
    S = (H * _E * H.transpose() + J * _model->_W_s * J.transpose())(0);
    // cout << "   S: " << S << endl;
    K = _E * H.transpose() / S;
    // cout << "   K: " << K.transpose() << endl;
    //  m_dist
    _mahalanobis = _r * _r / S;
    cout << "   mahalanobis: " << _mahalanobis << endl;
    if (_mahalanobis > tol)
      return 1;
    // update
    cout << "   p" << endl;
    _p = _p + K * _r;
    cout << "   E" << endl;
    _E = (_model->I - K * H) * _E;
    return 0; // no bp or aberrant measure
  }

  // attributes
  VectorXd _p; // temp
  MatrixXd _E; // temp
  Matrix<double, 1, Dynamic> _m;
  MatrixXd H, J, K;
  double _r, _mahalanobis, S;
  const Model *_model;
};

class Iekf {
public:
  // E is Sigma
  Iekf(Entity *entity) : _e(entity) {}

  int update(const VectorXd data, const VectorXd pose, const double tol) {
    // system propagation
    // cout << "E: " << _entity->E << endl;
    _e->E = _e->E + _e->m->_Q_a;
    _m0 = data({2, 3}); // adapt this shit to 3d
    _mk = _m0;
    // cout << "m: " << _m0 << endl;
    auto pk = _e->p;
    MatrixXd df;
    for (int j = 0; j < 10; j++) {
      // jacobians
      df = _e->m->_dfs(pk, pose, Matrix<double, 1, 4>({0, 0, _mk(0), _mk(1)}));
      H = df(all, _e->m->_dfs_parameter_indexes);
      J = df(all, _e->m->_dfs_measurement_error_indexes); // see dfsn_0
      _r =
          _e->m->_fs(pk, pose, Matrix<double, 1, 4>({0, 0, _mk(0), _mk(1)}))(0);
      // cout << "r: " << _r << endl;
      _dr = (J * (_m0 - _mk) + H * (_e->p - pk))(0);
      // cout << "dr: " << _dr << endl;
      S = (H * _e->E * H.transpose() + J * _e->m->_W_a * J.transpose())(0);
      // cout << "S: " << S << endl;
      if ((_e->E * H.transpose() / S * (_r + _dr)).array().isNaN().sum())
        break;
      K = _e->E * H.transpose() / S;
      pk = _e->p - K * (_r + _dr);
      if (_e->m->_safety(pk))
        break; // force safe range for parameters, break if unsafe
      _mk = _m0 - (_e->m->_W_a * J.transpose() / S) * (_r + _dr);
      // cout << "mk: " << mk.transpose() << endl;
      // cout << "pk: " << pk.transpose() << endl;
      // cout << "sum: " << _E.sum() << endl << endl;
      if (abs(_r) < tol)
        break;
    }
    // manual fix

    // cout << "assigning" << endl;
    _e->p = pk;
    // jacs
    df = _e->m->_dfs(_e->p, pose, Matrix<double, 1, 4>({0, 0, _mk(0), _mk(1)}));
    H = df(all, _e->m->_dfs_parameter_indexes);
    J = df(all, _e->m->_dfs_measurement_error_indexes);
    //
    L = _e->m->I - K * H;
    _e->E = L * _e->E * L.transpose() +
            K * J * _e->m->_W_a * J.transpose() * K.transpose();
    return 0;
  }

  // private:
  // Matrix<double, Dynamic, 12> df;
  // attributes
  Entity *_e;
  Matrix<double, Dynamic, 1> _mk, _m0; // inverted: see sophie
  MatrixXd H, J, K, df, L;
  double _r, _dr, S;
};

#endif // EKF_HPP

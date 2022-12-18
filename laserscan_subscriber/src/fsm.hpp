#ifndef FSM_HPP
#define FSM_HPP
#include "ekf.hpp"

class Fsm {
public:
  // functions
  Fsm(Model *m, Aggregate a) : _model(m), _a(a) {
    _a.m = (Model *)m;
    VectorXd p = m->_ls(a.pose, a.get_mat());
    MatrixXd E = m->_dop(p, a.pose, a.get_mat(), m->_dop_sigma);
    _ekf = new Ekf(m, p, E);
  }
  //~Fsm() { delete _ekf; }
  // state functions
  void f_flexible() {
    if (_ekf->update(_data.d, _data.p, _model->_mahalanobis_flex))
      state = s_fsm_sink; // goto sink
    else {
      _a.push_back(_data);
      state = s_fsm_strict; // goto strict next
    }
    return;
  }
  void f_strict() {
    if (_ekf->update(_data.d, _data.p, _model->_mahalanobis_strict)) {
      cout << "going to s_fsm_least_squares" << endl;
      // state = s_fsm_least_squares;
      f_least_squares(); // LS (it is called from here, state changed inside)
      return;
    }
    // else add data and stay in strict
    _a.push_back(_data);
  }
  void f_least_squares() {
    // here we LS, DOP and reset EKF
    _ekf->_p = _model->_ls(_a.pose, _a.get_mat());
    _ekf->_E =
        _model->_dop(_ekf->_p, _a.pose, _a.get_mat(), _model->_dop_sigma);
    // automatically updated in ekf! ... retesting
    if (_ekf->update(_data.d, _data.p, _model->_mahalanobis_strict)) {
      cout << "FSM CLOSING!!!!" << endl;
      state = s_fsm_close; // goto close next, what to do with data ..
    } else {
      // now add data to matrix
      _a.push_back(_data);
      state = s_fsm_strict; // goto strict next
    }
  }
  void f_close() {}
  void f_sink() {}
  // main function, outside loops over scan data (data: x,y,dist,angle)
  State process_measurement(Data data) {
    _data = data;
    // safety
     _model->_safety(_ekf->_p);
    switch (state) {
    case s_fsm_flexible:
      cout << "fsm flexible" << endl;
      f_flexible();
      break;
    case s_fsm_strict:
      cout << "fsm strict" << endl;
      f_strict();
      break;
    case s_fsm_least_squares:
      cout << "fsm ls" << endl;
      f_least_squares();
      break;
    case s_fsm_close:
      cout << "fsm close" << endl;
      f_close();
      break;
    case s_fsm_sink:
      cout << "fsm sink" << endl;
      f_sink();
      break;
    default:
      break;
    }
    return state;
  }
  // FOR USE WITH STL
  bool operator<(const Fsm &other_fsm) const {
    // assuming we are in flexible (TODO: account if not?)
    return (_ekf->_mahalanobis < other_fsm._ekf->_mahalanobis);
  }
  // attributes
  Model *_model;
  Aggregate _a;
  State state = s_fsm_flexible; // finite state machine state
  Data _data;
  unsigned short n = 0; // tested against _N in model
  Ekf *_ekf;            // init in state 1
};

#endif // FSM_HPP

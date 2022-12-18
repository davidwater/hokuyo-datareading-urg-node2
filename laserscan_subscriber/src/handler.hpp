#ifndef HANDLER_HPP
#define HANDLER_HPP
#include "fsm.hpp"

class Entity_map {
public:
  //** functions
  // returns lowest association if lower than thresh
  /*void associate_aggregate(Aggregate *a) {
    double min_mahalanobis = std::numeric_limits<double>::max();
    Entity *min_ptr = nullptr;
    for (auto &e : entities) {
      VectorXd _r = -e.m->_fs(e.p, a->pose, a->data);
      MatrixXd df = e.m->_dfs(e.p, a->pose, a->data);
      MatrixXd H = df(all, e.m->_dfs_parameter_indexes);
      MatrixXd J = df(all, e.m->_dfs_measurement_error_indexes);
      MatrixXd S = H * e.E * H.transpose() + J * e.m->_W_s * J.transpose();
      double mahalanobis =
          (_r.array() * _r.array() / S.diagonal().array()).sum() / _r.size();
      // check
      if (abs(mahalanobis) <= min_mahalanobis) {
        min_mahalanobis = abs(mahalanobis);
        min_ptr = &e;
      }
    }
    if (min_mahalanobis <= mahalanobis_aug) {
      cout << "*********ASSOCIATED !!!!!! ************" << min_mahalanobis
           << endl;
      a->e = min_ptr;
    } else {
      cout << "*********not associated :( ************" << min_mahalanobis
           << endl;
    }
  }*/
  // NEW !
  bool associate_data(Data &data) {
    cout << "associating :" << data.d.transpose() << endl;
    double min_mahalanobis = std::numeric_limits<double>::max();
    Entity *min_ptr = nullptr;
    for (auto &e : entities) {
      double _r = -e.m->_fs(e.p, data.p, Matrix<double, 1, Dynamic>(data.d))(0);
      MatrixXd df = e.m->_dfs(e.p, data.p, Matrix<double, 1, Dynamic>(data.d));
      MatrixXd H = df(all, e.m->_dfs_parameter_indexes);
      MatrixXd J = df(all, e.m->_dfs_measurement_error_indexes);
      double S = (H * e.E * H.transpose() + J * e.m->_W_s * J.transpose())(0);
      double mahalanobis = _r * _r / S;
      // check
      if (abs(mahalanobis) <= min_mahalanobis) {
        min_mahalanobis = abs(mahalanobis);
        min_ptr = &e;
      }
    }
    if (min_mahalanobis <= mahalanobis_aug) {
      cout << "*********ASSOCIATED !!!!!! ************" << min_mahalanobis
           << ", " << min_ptr->m->_model_index << endl;
      data.e = min_ptr;
      return 1;
    } else {
      cout << "*********not associated :( ************" << min_mahalanobis
           << endl;
      return 0;
    }
  }

  void augment_from_data(Data &data) {
    if (data.e == nullptr) // safety (can be removed)
      cout << "tried to augment with non-associated data !!!!" << endl;
    else {
      Iekf iekf(data.e);
      iekf.update(data.d, data.p, 1);
    }
  }

  // attributes
  vector<Entity> entities; // to be processed map
};

class Handler {
public:
  // non state functions
  Handler(const vector<Model *> models) : _models(models) {}
  vector<Data> preprocess_scan(Scan scan) {
    vector<Data> d_v;
    for (int i = 0; i < scan.data.rows(); i++) {
      VectorXd vbuf = scan.data(i, all);
      Data d(vbuf, scan.loc);
      if (map.associate_data(d))
        map.augment_from_data(d);
    }
    cout << "other way!!!" << endl;
    for (int i = scan.data.rows() - 1; i >= 0; i--) {
      VectorXd vbuf = scan.data(i, all);
      Data d(vbuf, scan.loc);
      if (map.associate_data(d))
        map.augment_from_data(d);
      else
        d_v.push_back(d); // only add non associated (2 passes)
    }
    return d_v;
  }
  // state functions
  void f_begin() {
    a.flush(_data);
    n = 1;
    state = s_continuous;
    fsms.clear();
  }
  void f_continuous() {
    if (a.check_continuity(_data)) {
      cout << "not continuous!" << endl;
      state = s_begin;
      return;
    }
    a.push_back(_data);
    if (n < N - 1)
      n++;
    else {
      state = s_fsm;
      f_fsm();
    }
  }
  void f_fsm() {
    // if they are empty, initialize them for all models
    if (fsms.empty())
      for (auto m : _models)
        fsms.push_back(Fsm(m, a));
    // instead check if existing to keep order
    // fsms_are_done.clear();
    for (auto &fsm : fsms) {

      State buf_state = fsm.process_measurement(_data);
      if (buf_state == s_fsm_close || buf_state == s_fsm_sink) {
        // check if not in fsms_done
        if (find(fsms_are_done.begin(), fsms_are_done.end(), &fsm) ==
            fsms_are_done.end())
          fsms_are_done.push_back(&fsm); // push back if not
      }
      // viz for debugging!
      /*else {
        if (fsm._model->_model_index == 0)
          v.add_ellipse(fsm._ekf->_p, "b-");
        if (fsm._model->_model_index == 1)
          v.add_line(fsm._ekf->_p, "b-");
      }*/
      //
    }
    if (fsms_are_done.size() == fsms.size()) // not all done
      f_close_fsms(); // we assume that the most accurate model has lowest r
  }
  void f_close_fsms() {
    Fsm *min_fsm = fsms_are_done.back();
    if (min_fsm != nullptr) {
      if (min_fsm->state > 3) { // TODO: make sure we are not adding sink fsms
        min_fsm->f_least_squares();
        map.entities.push_back(
            Entity(min_fsm->_a.m, min_fsm->_ekf->_p, min_fsm->_ekf->_E));
      }
    }

    // reset attributes
    fsms_are_done.clear();
    fsms.clear();
    n = 0;
    state = s_begin;
    // call f_begin as to not lose data point for new cycle
    f_begin();
  }
  void end_scan() {
    // stl find "minimal" fsm (see < in fsm)
    // first make a vector of only running fsms
    vector<Fsm> running_fsms;
    std::copy_if(
        fsms.begin(), fsms.end(), std::back_inserter(running_fsms),
        [](const Fsm &fsm) -> bool { return (fsm.state == s_fsm_strict); });
    // then least squares
    for (auto &fsm : running_fsms)
      fsm.f_least_squares();
    // then find the minimum
    vector<Fsm>::iterator min_it =
        min_element(running_fsms.begin(), running_fsms.end());
    if (min_it != running_fsms.end()) {
      map.entities.push_back(
          Entity(min_it->_model, min_it->_ekf->_p, min_it->_ekf->_E));
    }

    // reset attributes
    fsms_are_done.clear();
    fsms.clear();
    n = 0;
    state = s_begin;
  }

  // main functions
  State process_measurement(Data &data) {
    _data = data;
    // try associating and augmenting (since not associated)
    if (map.associate_data(data)) {
      map.augment_from_data(data);
      state = s_begin;
      return state;
    }
    //
    switch (state) {
    case s_begin:
      cout << "begin nsm" << endl;
      f_begin();
      break;
    case s_continuous:
      cout << "nsm continuous" << endl;
      f_continuous();
      break;
    case s_fsm:
      cout << "nsm fsm" << endl;
      f_fsm();
      break;
    default:;
    }
    return state;
  }
  // attributes
  const vector<Model *> _models; // permanent
  // VectorXd _data, _pose;         // buffers, pose should stay during scan
  Data _data;
  unsigned short n = 0;
  State state = s_begin;
  Entity_map map;
  Aggregate a; // buffer aggregate
  vector<Fsm> fsms;
  vector<Fsm *> fsms_are_done;
};

#endif // HANDLER_HPP

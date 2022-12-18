#ifndef MODEL_HPP
#define MODEL_HPP
#include "functions.hpp"
// general model
class Model {
public:
  // functions
  Model(VectorXd (*fs)(const VectorXd p, const VectorXd pos, const MatrixXd m),
        MatrixXd (*dfs)(const VectorXd p, const VectorXd pos, const MatrixXd m),
        VectorXd dfs_parameter_indexes, VectorXd dfs_measurement_error_indexes,
        MatrixXd (*dop)(const VectorXd p, const VectorXd pos, const MatrixXd m,
                        const double sigma),
        VectorXd (*ls)(const VectorXd loc, const MatrixXd data),
        bool (*safety)(VectorXd &p), MatrixXd W_a, MatrixXd W_s, MatrixXd Q_a,
        MatrixXd Q_s, double mahalanobis_strict, double mahalanobis_flex,
        unsigned short parameter_count, double dop_sigma,
        unsigned short model_index) {
    _fs = fs;
    _dfs = dfs;
    _dop = dop;
    _ls = ls;
    _safety = safety;
    _W_s = W_s;
    _W_a = W_a;
    _Q_s = Q_s;
    _Q_a = Q_a;
    _mahalanobis_strict = mahalanobis_strict;
    _mahalanobis_flex = mahalanobis_flex;
    _parameter_count = parameter_count;
    _model_index = model_index;
    _dfs_parameter_indexes = dfs_parameter_indexes;
    _dfs_measurement_error_indexes = dfs_measurement_error_indexes;
    I = MatrixXd::Identity(_parameter_count, _parameter_count);
    _dop_sigma = dop_sigma;
  }

  // attributes defining a model
  VectorXd (*_fs)(const VectorXd p, const VectorXd pos, const MatrixXd m);
  MatrixXd (*_dfs)(const VectorXd p, const VectorXd pos, const MatrixXd m);
  VectorXd _dfs_parameter_indexes;
  VectorXd _dfs_measurement_error_indexes;
  MatrixXd (*_dop)(const VectorXd p, const VectorXd pos, const MatrixXd m,
                   const double sigma);
  VectorXd (*_ls)(const VectorXd loc, const MatrixXd data);
  bool (*_safety)(VectorXd &p);
  MatrixXd _W_a, _W_s, _Q_a, _Q_s, I;
  double _mahalanobis_strict, _mahalanobis_flex,
      _dop_sigma; // thresholds
  unsigned short _parameter_count, _model_index;
};
//******* model independant parameters *******//
// continuity
const double angle_tolerance = 0.01, dist_tolerance = 0.3;
const unsigned short N = 10;
// association mahalanobis
double mahalanobis_aug = 1;
//********************************************//
// individual entity
class Entity {
public:
  Entity(Model *model, VectorXd _p, MatrixXd _E) : m(model), p(_p), E(_E) {}
  Model *m;
  VectorXd p; // parameters
  MatrixXd E; // uncertainty matrix, initialized with DOP
};
// one measurement
class Data {
public:
  Data() {}
  Data(VectorXd data, VectorXd pose) {
    d = data;
    p = pose;
  }

  // attributes
  VectorXd d;          // data (measurement)
  VectorXd p;          // pose at measurement
  Entity *e = nullptr; // associated entity
};
// aggregate to be combined
class Aggregate {
public:
  // functions
  /*void flush(VectorXd _data, VectorXd _pose) { // flush and fill with one data
    data.conservativeResize(1, _data.size());  // resetting
    data.row(0) = _data;
    pose = _pose;
    e = nullptr;
    m = nullptr;
  }
  void push_back(VectorXd v) {
    data.conservativeResize(data.rows() + 1, NoChange);
    data(last, all) = v;
  }

  // cont check between new data vector v and last data in aggregate
  bool check_continuity(VectorXd v) {
    double dist_variation = abs(v(2) - data(last, 2));
    double angle_variation =
        atan2(sin(v(3) - data(last, 3)), cos(v(3) - data(last, 3)));
    // double angle_variation = abs(v(3) - data(last, 3));
    //  ret 1 for error
    //  PRINT FOR DEBUG
    if (angle_variation > angle_tolerance || dist_variation > dist_tolerance)
      cout << "Dist variation: " << dist_variation << endl
           << "Angle variation: " << angle_variation << endl;
    return (angle_variation > angle_tolerance ||
            dist_variation > dist_tolerance);
  }
  */
  MatrixXd get_mat() { // returns a matrix of measurements (for ls/dop)
    MatrixXd m;
    m.conservativeResize(data_v.size(), 4); // x,y,d,an
    for (int i = 0; i < data_v.size(); i++)
      m.row(i) = data_v[i].d;
    return m; // optimize this fun?
  }
  void push_back(Data data) { data_v.push_back(data); }
  bool check_continuity(Data data) {
    double dist_variation = abs(data.d(2) - data_v.back().d(2));
    double angle_variation = atan2(sin(data.d(3) - data_v.back().d(3)),
                                   cos(data.d(3) - data_v.back().d(3)));
    // double angle_variation = abs(v(3) - data(last, 3));
    //  ret 1 for error
    //  PRINT FOR DEBUG
    if (angle_variation > angle_tolerance || dist_variation > dist_tolerance)
      cout << "Dist variation: " << dist_variation << endl
           << "Angle variation: " << angle_variation << endl;
    return (angle_variation > angle_tolerance ||
            dist_variation > dist_tolerance);
  }
  void flush(Data data) { // flush and fill with one data
    data_v.clear();       // resetting
    data_v.push_back(data);
    pose = data.p;
    // e = nullptr;
    m = nullptr;
  }

  // attributes
  Model *m = nullptr;
  // Entity *e = nullptr;
  //  MatrixXd data; // does not include pose
  VectorXd pose; // not necessary
  vector<Data> data_v;
};

// states of finite state machine and nested state machin
enum State {
  s_begin,      // 0
  s_continuous, // 1
  // s_association,       // 2
  s_fsm,               // 3 -> 2
  s_fsm_flexible,      // 3a -> 2a ..
  s_fsm_strict,        // 3b
  s_fsm_least_squares, // 3c
  s_fsm_close,         // 3d
  s_fsm_sink,          // 3e
  s_merge              // 4 -> 3
};

#endif // MODEL_HPP

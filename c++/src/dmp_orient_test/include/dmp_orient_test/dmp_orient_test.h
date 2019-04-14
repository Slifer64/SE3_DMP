#ifndef DMP_ORIENT_TEST_H
#define DMP_ORIENT_TEST_H

#include <string>
#include <vector>
#include <memory>
#include <armadillo>

#include <dmp_lib/CanonicalClock/CanonicalClock.h>
#include <dmp_lib/GatingFunction/GatingFunction.h>
#include <dmp_lib/DMP/DMP_orient.h>

class DMP_orient_test
{

public:
  DMP_orient_test();

private:

  void readParams();
  void readTrainingData();
  void train();
  void simulate();
  void saveSimData();

  // DMP related params
  std::string train_method;
  arma::Col<int> N_kernels;
  double a_z;
  double b_z;
  std::shared_ptr<as64_::CanonicalClock> can_clock_ptr;
  std::shared_ptr<as64_::GatingFunction> shape_attr_gat_ptr;
  std::shared_ptr<as64_::DMP_orient> dmp_o;
  std::string shape_attr_gat_type;
  double s0, send;

  // training data
  std::string train_data_filename; // name of the file containing the training data
  arma::rowvec Timed;
  arma::mat Qd_data;
  arma::mat vRotd_data;
  arma::mat dvRotd_data;

  // simulation data
  std::string sim_data_filename; // name of the file where the simulation data will be stored
  arma::rowvec Time;
  arma::mat Q_data;
  arma::mat vRot_data;
  arma::mat dvRot_data;

  arma::wall_clock timer;

  static arma::vec quatProd(const arma::vec &quat1, const arma::vec &quat2);
  static arma::vec quatExp(const arma::vec &v_rot, double zero_tol=1e-16);

};

#endif

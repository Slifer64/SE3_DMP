#ifndef DMP_pos_test_H
#define DMP_pos_test_H

#include <string>
#include <vector>
#include <memory>
#include <armadillo>

#include <dmp_lib/CanonicalClock/CanonicalClock.h>
#include <dmp_lib/GatingFunction/GatingFunction.h>
#include <dmp_lib/DMP/DMP_pos.h>

class DMP_pos_test
{

public:
  DMP_pos_test();

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
  std::shared_ptr<as64_::DMP_pos> dmp_p;
  std::string shape_attr_gat_type;
  double s0, send;

  // training data
  std::string train_data_filename; // name of the file containing the training data
  arma::rowvec Timed;
  arma::mat Pd_data;
  arma::mat dPd_data;
  arma::mat ddPd_data;

  // simulation data
  std::string sim_data_filename; // name of the file where the simulation data will be stored
  arma::rowvec Time;
  arma::mat P_data;
  arma::mat dP_data;
  arma::mat ddP_data;

  arma::wall_clock timer;

};

#endif

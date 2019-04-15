#ifndef SE3_DMP_H
#define SE3_DMP_H

#include <string>
#include <vector>
#include <memory>
#include <armadillo>

#include <dmp_lib/CanonicalClock/CanonicalClock.h>
#include <dmp_lib/GatingFunction/GatingFunction.h>
#include <dmp_lib/DMP/DMP_pos.h>
#include <dmp_lib/DMP/DMP_orient.h>

#include <se3_dmp/robot/robot.h>

class SE3_DMP
{

public:
  SE3_DMP();

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
  std::shared_ptr<as64_::DMP_orient> dmp_o;
  std::string shape_attr_gat_type;
  double s0, send;

  // training data
  std::string train_data_filename; // name of the file containing the training data
  arma::vec q0;
  arma::rowvec Timed;
  arma::mat Pd_data;
  arma::mat dPd_data;
  arma::mat ddPd_data;
  arma::mat Qd_data;
  arma::mat vRotd_data;
  arma::mat dvRotd_data;

  // simulation data
  std::string sim_data_filename; // name of the file where the simulation data will be stored
  arma::rowvec Time;
  arma::mat P_data;
  arma::mat dP_data;
  arma::mat ddP_data;
  arma::mat Q_data;
  arma::mat vRot_data;
  arma::mat dvRot_data;

  arma::wall_clock timer;

  // robot
  std::string robot_type;
  std::shared_ptr<Robot> robot;

  static arma::vec quatProd(const arma::vec &quat1, const arma::vec &quat2);
  static arma::vec quatExp(const arma::vec &v_rot, double zero_tol=1e-16);

};

#endif // SE3_DMP_H

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

#include <se3_dmp/gui/mainwindow.h>

class SE3_DMP
{

public:
  SE3_DMP();

  void run();

private:
  void launchGUI();
  bool gotoStartPose();

  void readParams();
  bool loadTrainingData(const std::string &path);
  bool train();
  void simulate();
  bool saveExecData(const std::string &save_path="");
  void clearData();

  void setMode(Robot::Mode mode);

  arma::vec q_start; // start pose

  std::string err_msg;
  void setErrMsg(const std::string &msg) { err_msg = msg; }
  std::string getErrMsg() const { return err_msg; }

  std::string info_msg;
  void setInfoMsg(const std::string &msg) { info_msg = msg; }
  std::string getInfoMsg() const { return info_msg; }

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
  bool is_trained;

  bool controller_finished;

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

  // GUI
  MainWindow *gui;

  Semaphore start_sem;
  Semaphore finish_sem;

  static arma::vec quatProd(const arma::vec &quat1, const arma::vec &quat2);
  static arma::vec quatExp(const arma::vec &v_rot, double zero_tol=1e-16);

};

#endif // SE3_DMP_H

#include <se3_dmp/se3_dmp.h>

#include <exception>
#include <fstream>
#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>

#include <dmp_lib/GatingFunction/LinGatingFunction.h>
#include <dmp_lib/GatingFunction/ExpGatingFunction.h>
#include <dmp_lib/GatingFunction/SigmoidGatingFunction.h>

#include <se3_dmp/robot/lwr4p_robot.h>

#include <io_lib/io_utils.h>

using namespace as64_;

SE3_DMP::SE3_DMP()
{
  std::cerr << "[SE3_DMP::SE3_DMP]: readParams() ...\n";
  readParams();

  std::cerr << "[SE3_DMP::SE3_DMP]: readTrainingData() ...\n";
  readTrainingData();

  std::cerr << "[SE3_DMP::SE3_DMP]: train() ...\n";
  train();

  if (robot_type.compare("lwr4p")==0) robot.reset(new LWR4p_Robot());
  else throw std::runtime_error("Unsupported robot type \"" + robot_type + "\".");

  std::cerr << "[SE3_DMP::SE3_DMP]: simulate() ...\n";
  simulate();

  std::cerr << "[SE3_DMP::SE3_DMP]: saveSimData() ...\n";
  saveSimData();

}

void SE3_DMP::readParams()
{
  ros::NodeHandle nh("~");

  if (!nh.getParam("robot_type", robot_type)) throw std::ios_base::failure("[SE3_DMP::readParams]: Failed to read \"robot_type\" param.");

  if (!nh.getParam("train_data_filename", train_data_filename)) throw std::ios_base::failure("[SE3_DMP::readParams]: Failed to read \"train_data_filename\" param.");
  if (!nh.getParam("sim_data_filename", sim_data_filename)) throw std::ios_base::failure("[SE3_DMP::readParams]: Failed to read \"sim_data_filename\" param.");

  if (!nh.getParam("a_z", a_z)) throw std::ios_base::failure("[SE3_DMP::readParams]: Failed to read \"a_z\" param.");
  if (!nh.getParam("b_z", b_z)) b_z = a_z/4;
  if (!nh.getParam("train_method", train_method)) throw std::ios_base::failure("[SE3_DMP::readParams]: Failed to read \"train_method\" param.");

  if (!nh.getParam("shape_attr_gat_type", shape_attr_gat_type)) throw std::ios_base::failure("[SE3_DMP::readParams]: Failed to read \"shape_attr_gat_type\" param.");

  if (!nh.getParam("s0", s0)) throw std::ios_base::failure("[SE3_DMP::readParams]: Failed to read \"s0\" param.");
  if (!nh.getParam("send", send)) throw std::ios_base::failure("[SE3_DMP::readParams]: Failed to read \"send\" param.");

  std::vector<int> n_ker;
  if (!nh.getParam("N_kernels", n_ker)) throw std::ios_base::failure("Failed to read \"N_kernels\" param.");
  N_kernels = n_ker;
}

void SE3_DMP::readTrainingData()
{
  std::string path = ros::package::getPath("SE3_DMP") + "/data/" + train_data_filename;

  std::ifstream in(path.c_str(), std::ios::in);
  if (!in) throw std::ios_base::failure("[SE3_DMP::readTrainingData]: Failed to open file \"" + path + "\".");

  io_::read_mat(q0, in, true);
  io_::read_mat(Timed, in, true);
  io_::read_mat(Pd_data, in, true);
  io_::read_mat(dPd_data, in, true);
  io_::read_mat(ddPd_data, in, true);
  io_::read_mat(Qd_data, in, true);
  io_::read_mat(vRotd_data, in, true);
  io_::read_mat(dvRotd_data, in, true);

  in.close();
}

void SE3_DMP::train()
{
  can_clock_ptr.reset(new CanonicalClock());

  if (shape_attr_gat_type.compare("Linear")==0) shape_attr_gat_ptr.reset(new LinGatingFunction(s0,send));
  else if (shape_attr_gat_type.compare("Exp")==0) shape_attr_gat_ptr.reset(new ExpGatingFunction(s0,send));
  else if (shape_attr_gat_type.compare("Sigmoid")==0) shape_attr_gat_ptr.reset(new SigmoidGatingFunction(s0,send));
  else throw std::runtime_error("[SE3_DMP::train]: Unsupported shapre attractor gating \"" + shape_attr_gat_type + "\".");

  dmp_::TRAIN_METHOD tr_m;
  if (train_method.compare("LWR")==0) tr_m = dmp_::LWR;
  else if (train_method.compare("LS")==0) tr_m = dmp_::LS;
  else throw std::runtime_error("[SE3_DMP::train]: Unsupported training method \"" + train_method + "\".");

  dmp_p.reset(new DMP_pos(N_kernels, a_z, b_z, can_clock_ptr, shape_attr_gat_ptr));
  dmp_o.reset(new DMP_orient(N_kernels, a_z, b_z, can_clock_ptr, shape_attr_gat_ptr));

  std::cout << "Position DMP training...\n";
  timer.tic();
  arma::vec offline_train_mse_p;
  dmp_p->train(tr_m, Timed, Pd_data, dPd_data, ddPd_data, &offline_train_mse_p);
  std::cout << "offline_train_mse_p = \n" << offline_train_mse_p << "\n";
  std::cout << "Elapsed time " << timer.toc() << " sec\n";

  std::cout << "Orient DMP training...\n";
  timer.tic();
  arma::mat offline_train_mse_o;
  dmp_o->train(tr_m, Timed, Qd_data, vRotd_data, dvRotd_data, &offline_train_mse_o);
  std::cout << "offline_train_mse_o = \n" << offline_train_mse_o << "\n";
  std::cout << "Elapsed time " << timer.toc() << " sec\n";

}

void SE3_DMP::simulate()
{
  int i_end = Pd_data.n_cols - 1;
  arma::vec P0 = Pd_data.col(0);
  arma::vec Pg = Pd_data.col(i_end);
  arma::vec Q0 = Qd_data.col(0);
  arma::vec Qg = Qd_data.col(i_end);
  double T = Timed(i_end);
  double dt = Timed(1) - Timed(0);

  // set initial values
  double t = 0.0;
  double x = 0.0;
  double dx = 0.0;

  arma::vec P = P0;
  arma::vec dP = arma::vec().zeros(3);
  arma::vec ddP = arma::vec().zeros(3);

  arma::vec Q = Q0;
  arma::vec vRot = arma::vec().zeros(3);
  arma::vec dvRot = arma::vec().zeros(3);

  double t_end = T;
  can_clock_ptr->setTau(t_end);

  int iters = 0;
  Time.clear();
  P_data.clear();
  dP_data.clear();
  ddP_data.clear();
  Q_data.clear();
  vRot_data.clear();
  dvRot_data.clear();

  // simulate
  while (true)
  {
    // data logging
    Time = arma::join_horiz(Time, arma::vec({t}));
    P_data = arma::join_horiz(P_data, P);
    dP_data = arma::join_horiz(dP_data, dP);
    ddP_data = arma::join_horiz(ddP_data, ddP);
    Q_data = arma::join_horiz(Q_data, Q);
    vRot_data = arma::join_horiz(vRot_data, vRot);
    dvRot_data = arma::join_horiz(dvRot_data, dvRot);

    // DMP simulation
    arma::vec Z_c = arma::vec().zeros(3);
    ddP = dmp_p->getAccel(x, P, dP, P0, Pg, Z_c);
    dvRot = dmp_o->getRotAccel(x, Q, vRot, Q0, Qg, Z_c);

    // Update phase variable
    dx = can_clock_ptr->getPhaseDot(x);

    // Stopping criteria
    if (t>=t_end) break;

    // Numerical integration
    iters++;
    t = t + dt;
    x = x + dx*dt;
    P = P + dP*dt;
    dP = dP + ddP*dt;
    Q = quatProd( quatExp(vRot*dt), Q);
    vRot = vRot + dvRot*dt;
  }
}

void SE3_DMP::saveSimData()
{
  std::string path = ros::package::getPath("SE3_DMP") + "/data/" + sim_data_filename;

  std::cerr << "[SE3_DMP::saveSimData]: Saving data to \"" << path << "\"...\n";
  std::ofstream out(path.c_str(), std::ios::out);
  if (!out) throw std::ios_base::failure("[SE3_DMP::saveSimData]: Failed to create file \"" + path + "\".");

  io_::write_scalar(static_cast<double>(a_z), out, true);
  io_::write_mat(Time, out, true);
  io_::write_mat(P_data, out, true);
  io_::write_mat(dP_data, out, true);
  io_::write_mat(ddP_data, out, true);
  io_::write_mat(Q_data, out, true);
  io_::write_mat(vRot_data, out, true);
  io_::write_mat(dvRot_data, out, true);

  out.close();

  std::cerr << "[SE3_DMP::saveSimData]: DONE!\n";
}

arma::vec SE3_DMP::quatProd(const arma::vec &quat1, const arma::vec &quat2)
{
  arma::vec quat12(4);

  double n1 = quat1(0);
  arma::vec e1 = quat1.subvec(1,3);

  double n2 = quat2(0);
  arma::vec e2 = quat2.subvec(1,3);

  quat12(0) = n1*n2 - arma::dot(e1,e2);
  quat12.subvec(1,3) = n1*e2 + n2*e1 + arma::cross(e1,e2);

  return quat12;
}

arma::vec SE3_DMP::quatExp(const arma::vec &v_rot, double zero_tol)
{
  arma::vec quat(4);
  double norm_v_rot = arma::norm(v_rot);
  double theta = norm_v_rot;

 if (norm_v_rot > zero_tol)
 {
    quat(0) = std::cos(theta/2);
    quat.subvec(1,3) = std::sin(theta/2)*v_rot/norm_v_rot;
  }
  else{
    quat << 1 << 0 << 0 << 0;
  }

  return quat;
}

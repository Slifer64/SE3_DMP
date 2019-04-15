#include <dmp_pos_test/dmp_pos_test.h>

#include <exception>
#include <fstream>
#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>

#include <dmp_lib/GatingFunction/LinGatingFunction.h>
#include <dmp_lib/GatingFunction/ExpGatingFunction.h>
#include <dmp_lib/GatingFunction/SigmoidGatingFunction.h>

#include <io_lib/io_utils.h>

using namespace as64_;

DMP_pos_test::DMP_pos_test()
{
  std::cerr << "[DMP_pos_test::DMP_pos_test]: readParams() ...\n";
  readParams();

  std::cerr << "[DMP_pos_test::DMP_pos_test]: readTrainingData() ...\n";
  readTrainingData();

  std::cerr << "[DMP_pos_test::DMP_pos_test]: train() ...\n";
  train();

  std::cerr << "[DMP_pos_test::DMP_pos_test]: simulate() ...\n";
  simulate();

  std::cerr << "[DMP_pos_test::DMP_pos_test]: saveSimData() ...\n";
  saveSimData();

}

void DMP_pos_test::readParams()
{
  ros::NodeHandle nh("~");

  if (!nh.getParam("train_data_filename", train_data_filename)) throw std::ios_base::failure("[DMP_pos_test::readParams]: Failed to read \"train_data_filename\" param.");
  if (!nh.getParam("sim_data_filename", sim_data_filename)) throw std::ios_base::failure("[DMP_pos_test::readParams]: Failed to read \"sim_data_filename\" param.");

  if (!nh.getParam("a_z", a_z)) throw std::ios_base::failure("[DMP_pos_test::readParams]: Failed to read \"a_z\" param.");
  if (!nh.getParam("b_z", b_z)) b_z = a_z/4;
  if (!nh.getParam("train_method", train_method)) throw std::ios_base::failure("[DMP_pos_test::readParams]: Failed to read \"train_method\" param.");

  if (!nh.getParam("shape_attr_gat_type", shape_attr_gat_type)) throw std::ios_base::failure("[DMP_pos_test::readParams]: Failed to read \"shape_attr_gat_type\" param.");

  if (!nh.getParam("s0", s0)) throw std::ios_base::failure("[DMP_pos_test::readParams]: Failed to read \"s0\" param.");
  if (!nh.getParam("send", send)) throw std::ios_base::failure("[DMP_pos_test::readParams]: Failed to read \"send\" param.");

  std::vector<int> n_ker;
  if (!nh.getParam("N_kernels", n_ker)) throw std::ios_base::failure("Failed to read \"N_kernels\" param.");
  N_kernels = n_ker;
}

void DMP_pos_test::readTrainingData()
{
  std::string path = ros::package::getPath("dmp_pos_test") + "/data/" + train_data_filename;

  std::ifstream in(path.c_str(), std::ios::in);
  if (!in) throw std::ios_base::failure("[DMP_pos_test::readTrainingData]: Failed to open file \"" + path + "\".");

  io_::read_mat(Timed, in, true);
  io_::read_mat(Pd_data, in, true);
  io_::read_mat(dPd_data, in, true);
  io_::read_mat(ddPd_data, in, true);

  in.close();
}

void DMP_pos_test::train()
{
  can_clock_ptr.reset(new CanonicalClock());

  if (shape_attr_gat_type.compare("Linear")==0) shape_attr_gat_ptr.reset(new LinGatingFunction(s0,send));
  else if (shape_attr_gat_type.compare("Exp")==0) shape_attr_gat_ptr.reset(new ExpGatingFunction(s0,send));
  else if (shape_attr_gat_type.compare("Sigmoid")==0) shape_attr_gat_ptr.reset(new SigmoidGatingFunction(s0,send));
  else throw std::runtime_error("[DMP_pos_test::train]: Unsupported shapre attractor gating \"" + shape_attr_gat_type + "\".");

  dmp_p.reset(new DMP_pos(N_kernels, a_z, b_z, can_clock_ptr, shape_attr_gat_ptr));

  dmp_::TRAIN_METHOD tr_m;
  if (train_method.compare("LWR")==0) tr_m = dmp_::LWR;
  else if (train_method.compare("LS")==0) tr_m = dmp_::LS;
  else throw std::runtime_error("[DMP_pos_test::train]: Unsupported training method \"" + train_method + "\".");

  std::cout << "Position DMP training...\n";
  timer.tic();
  arma::vec offline_train_mse;
  dmp_p->train(tr_m, Timed, Pd_data, dPd_data, ddPd_data, &offline_train_mse);
  std::cout << "offline_train_mse = \n" << offline_train_mse << "\n";
  std::cout << "Elapsed time " << timer.toc() << " sec\n";

}

void DMP_pos_test::simulate()
{
  int i_end = Pd_data.n_cols - 1;
  arma::vec P0 = Pd_data.col(0);
  arma::vec Pg = Pd_data.col(i_end);
  double T = Timed(i_end);
  double dt = Timed(1) - Timed(0);

  // set initial values
  double t = 0.0;
  double x = 0.0;
  double dx = 0.0;

  arma::vec P = P0;
  arma::vec dP = arma::vec().zeros(3);
  arma::vec ddP = arma::vec().zeros(3);

  double t_end = T;
  can_clock_ptr->setTau(t_end);

  int iters = 0;
  Time.clear();
  P_data.clear();
  dP_data.clear();
  ddP_data.clear();

  // simulate
  while (true)
  {
    // data logging
    Time = arma::join_horiz(Time, arma::vec({t}));
    P_data = arma::join_horiz(P_data, P);
    dP_data = arma::join_horiz(dP_data, dP);
    ddP_data = arma::join_horiz(ddP_data, ddP);

    // DMP simulation
    arma::vec Z_c = arma::vec().zeros(3);
    ddP = dmp_p->getAccel(x, P, dP, P0, Pg, Z_c);

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
  }
}

void DMP_pos_test::saveSimData()
{
  std::string path = ros::package::getPath("dmp_pos_test") + "/data/" + sim_data_filename;

  std::cerr << "[DMP_pos_test::saveSimData]: Saving data to \"" << path << "\"...\n";
  std::ofstream out(path.c_str(), std::ios::out);
  if (!out) throw std::ios_base::failure("[DMP_pos_test::saveSimData]: Failed to create file \"" + path + "\".");

  io_::write_scalar(static_cast<double>(a_z), out, true);
  io_::write_mat(Time, out, true);
  io_::write_mat(P_data, out, true);
  io_::write_mat(dP_data, out, true);
  io_::write_mat(ddP_data, out, true);

  out.close();

  std::cerr << "[DMP_pos_test::saveSimData]: DONE!\n";
}

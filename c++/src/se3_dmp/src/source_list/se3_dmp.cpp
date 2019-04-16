#include <se3_dmp/se3_dmp.h>

#include <QApplication>

#include <exception>
#include <fstream>
#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>

#include <dmp_lib/GatingFunction/LinGatingFunction.h>
#include <dmp_lib/GatingFunction/ExpGatingFunction.h>
#include <dmp_lib/GatingFunction/SigmoidGatingFunction.h>

#include <se3_dmp/robot/lwr4p_robot.h>
#include <se3_dmp/robot/lwr4p_sim_robot.h>

#include <io_lib/io_utils.h>

using namespace as64_;

SE3_DMP::SE3_DMP()
{
  is_trained = false;

  ros::NodeHandle("~").getParam("robot_type",robot_type);
  if (robot_type.compare("lwr4p")==0) robot.reset(new LWR4p_Robot);
  if (robot_type.compare("lwr4p_sim")==0) robot.reset(new LWR4p_Sim_Robot);
  else throw std::runtime_error("Unsupported robot type \"" + robot_type + "\".");

  launchGUI();
}

void SE3_DMP::launchGUI()
{
  std::thread([this]()
              {
                int argc = 0;
                char **argv = 0;
                QApplication app(argc, argv);
                this->gui = new MainWindow(this->robot.get());
                this->gui->show();
                this->start_sem.notify();
                app.exec();
                std::cerr << "[SE3_DMP::launchGUI]: Notifying!\n";
                this->finish_sem.notify();
                delete (this->gui); // must be destructed in this thread!
              }).detach();

  start_sem.wait(); // wait for gui to be initialized
}

void SE3_DMP::run()
{
  q_start = robot->getJointsPosition();
  controller_finished = false;

  while (gui->isRunning())
  {
    // =======> Check mode
    if (gui->getMode()==MainWindow::FREEDRIVE && robot->getMode()!=Robot::FREEDRIVE) setMode(Robot::FREEDRIVE);
    else if (gui->getMode()==MainWindow::IDLE && robot->getMode()!=Robot::IDLE) setMode(Robot::IDLE);
    else if (gui->getMode()==MainWindow::RUN_CONTROLLER && robot->getMode()!=Robot::CART_VEL_CTRL)
    {
      setMode(Robot::CART_VEL_CTRL);
      controller_finished = false;
      if (!is_trained)
      {
        gui->notTrainedSignal("Cannot run the controller!\n The model is untrained...");
      }
      else std::thread(&SE3_DMP::simulate, this).detach();
    }

    if (gui->getMode()==MainWindow::RUN_CONTROLLER && controller_finished)
    {
      controller_finished = false;
      gui->controllerFinishedSignal("The controller has finished execution!");
    }

    // =======> Check if robot is ok
    if (!robot->isOk())
    {
      gui->terminateAppSignal("An error occured on the robot.\nThe program will terminate...");
      break;
    }

    if (gui->gotoStartPose())
    {
      if (gotoStartPose()) gui->sendGotoStartPoseAck(true, "Reached start pose!");
      else gui->sendGotoStartPoseAck(false, "Failed to reach start pose...");
    }

    if (gui->loadData())
    {
      if (! loadTrainingData(gui->getLoadDataPath()) ) gui->sendLoadAck(false, getErrMsg().c_str());
      else gui->sendLoadAck(true, "The training data were successfully loaded!");
    }

    if (gui->saveData())
    {
      if (! saveExecData(gui->getSaveDataPath()) ) gui->sendSaveAck(false, getErrMsg().c_str());
      else gui->sendSaveAck(true, "The execution data were successfully saved!");
    }

    if (gui->trainModel())
    {
      if (! train() ) gui->sendTrainAck(false, getErrMsg().c_str());
      else gui->sendTrainAck(true, QString("The model was trained!\n") + getInfoMsg().c_str());
    }

    if (gui->currentPoseAsStart())
    {
      q_start = robot->getJointsPosition();
      gui->sendSetStartPoseAck(true, "Registered current pose as start!");
    }

    // robot->update();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  if (robot->isOk()) robot->stop();

  std::cerr << "[SE3_DMP::run]: Waiting to be notified...\n";
  finish_sem.wait(); // wait for gui to finish
  std::cerr << "[SE3_DMP::launchGUI]: Got notification!\n";

}

void SE3_DMP::setMode(Robot::Mode mode)
{
  robot->setMode(mode);
  if (robot->isOk()) gui->modeChangedSignal(); // inform the gui that the robot's mode changed
}

bool SE3_DMP::gotoStartPose()
{
  robot->update();
  arma::vec q = robot->getJointsPosition();
  double duration = arma::max(arma::abs(q-q_start))*8.0/3.14159;
  return robot->setJointsTrajectory(q_start, duration);
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

bool SE3_DMP::loadTrainingData(const std::string &path)
{
  std::ifstream in(path.c_str(), std::ios::in);
  if (!in)
  {
    // throw std::ios_base::failure("Failed to open file \"" + path + "\".");
    setErrMsg("Failed to open file \"" + path + "\".");
    return false;
  }

  try
  {
    io_::read_mat(q0, in, true);
    io_::read_mat(Timed, in, true);
    io_::read_mat(Pd_data, in, true);
    io_::read_mat(dPd_data, in, true);
    io_::read_mat(ddPd_data, in, true);
    io_::read_mat(Qd_data, in, true);
    io_::read_mat(vRotd_data, in, true);
    io_::read_mat(dvRotd_data, in, true);

    in.close();
    return true;
  }
  catch (std::exception &e)
  {
    setErrMsg("Error reading training data!\nMaybe the file is corrupted...");
    in.close();
    return false;
  }
}

bool SE3_DMP::train()
{
  is_trained = false;
  readParams();

  std::ostringstream info_msg_stream;

  if (Timed.size() == 0)
  {
    setErrMsg("The training data are empty...");
    return false;
  }

  try{
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

    info_msg_stream << "=== Training error ==\n"
                    << "Cart Position DMP:\n"
                    << offline_train_mse_p.t()
                    << "Orientation DMP:\n"
                    << offline_train_mse_o;

    setInfoMsg(info_msg_stream.str());

    is_trained = true;
    return true;
  }
  catch (std::exception &e)
  {
    setErrMsg(std::string("Error during training:\n") + e.what());
    return false;
  }
}

void SE3_DMP::simulate()
{
  int i_end = Pd_data.n_cols - 1;

  arma::vec P0 = robot->getTaskPosition();
  arma::vec Q0 = robot->getTaskOrientation();

  // arma::vec P0 = Pd_data.col(0);
  // arma::vec Q0 = Qd_data.col(0);

  arma::vec Pg = Pd_data.col(i_end);
  arma::vec Qg = Qd_data.col(i_end);

  double T = Timed(i_end);
  double dt = robot->getCtrlCycle();

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
  while (robot->isOk() && gui->getMode()==MainWindow::RUN_CONTROLLER)
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

    arma::vec V_cmd = arma::join_vert(dP, vRot);
    robot->setTaskVelocity(V_cmd);
    robot->update();
  }

  controller_finished = true;
}

bool SE3_DMP::saveExecData(const std::string &save_path)
{
  if (Time.size() == 0)
  {
    setErrMsg("The execution data are empty!");
    return false;
  }

  // std::string path = save_path.empty() ? ros::package::getPath("SE3_DMP") + "/data/" + sim_data_filename : save_path;
  std::ofstream out(save_path.c_str(), std::ios::out);
  if (!out)
  {
    setErrMsg("Failed to create file \"" + save_path + "\".");
    return false;
  } //throw std::ios_base::failure("[SE3_DMP::saveExecData]: Failed to create file \"" + save_path + "\".");

  try
  {
    io_::write_scalar(static_cast<double>(a_z), out, true);
    io_::write_mat(Time, out, true);
    io_::write_mat(P_data, out, true);
    io_::write_mat(dP_data, out, true);
    io_::write_mat(ddP_data, out, true);
    io_::write_mat(Q_data, out, true);
    io_::write_mat(vRot_data, out, true);
    io_::write_mat(dvRot_data, out, true);
    out.close();
    return true;
  }
  catch (std::exception &e)
  {
    setErrMsg(std::string("Error writing data:\n") + e.what());
    return false;
  }


}

void SE3_DMP::clearData()
{
  Time.clear();
  P_data.clear();
  dP_data.clear();
  ddP_data.clear();
  Q_data.clear();
  vRot_data.clear();
  dvRot_data.clear();
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

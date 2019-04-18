#include <se3_dmp/se3_dmp.h>

#include <QApplication>

#include <exception>
#include <fstream>
#include <iostream>
#include <algorithm>

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
  else if (robot_type.compare("lwr4p_sim")==0) robot.reset(new LWR4p_Sim_Robot);
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
  double duration = std::max(arma::max(arma::abs(q-q_start))*8.0/3.14159, 2.5);
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

  std::vector<double> temp;
  // === Cartesian spring-damper params ===
  if (!nh.getParam("Mp", temp)) throw std::ios_base::failure("[SE3_DMP::readParams]: Failed to read \"Mp\" param.");
  Mp = temp;
  if (!nh.getParam("Kp", temp)) throw std::ios_base::failure("[SE3_DMP::readParams]: Failed to read \"Kp\" param.");
  Kp = temp;
  if (!nh.getParam("Dp", temp)) Dp = 2*arma::sqrt( Mp % Kp);
  else Dp = temp;

  // === Orientation spring-damper params ===
  if (!nh.getParam("Mo", temp)) throw std::ios_base::failure("[SE3_DMP::readParams]: Failed to read \"Mo\" param.");
  Mo = temp;
  if (!nh.getParam("Ko", temp)) throw std::ios_base::failure("[SE3_DMP::readParams]: Failed to read \"Ko\" param.");
  Ko = temp;
  if (!nh.getParam("Do", temp)) Do = 2*arma::sqrt( Mo % Ko);
  else Do = temp;

  if (!nh.getParam("Fext_dead_zone", temp)) Fext_dead_zone = arma::vec().zeros(6);
  else Fext_dead_zone = temp;

  if (!nh.getParam("a_fext_filt", a_fext_filt)) a_fext_filt = 0.0;

  // === DMP stopping params ===
  if (!nh.getParam("a_force", a_force)) throw std::ios_base::failure("[SE3_DMP::readParams]: Failed to read \"a_force\" param.");
  if (!nh.getParam("c_force", c_force)) throw std::ios_base::failure("[SE3_DMP::readParams]: Failed to read \"c_force\" param.");

  if (!nh.getParam("a_pos", a_pos)) throw std::ios_base::failure("[SE3_DMP::readParams]: Failed to read \"a_pos\" param.");
  if (!nh.getParam("c_pos", c_pos)) throw std::ios_base::failure("[SE3_DMP::readParams]: Failed to read \"c_pos\" param.");

  if (!nh.getParam("a_orient", a_orient)) throw std::ios_base::failure("[SE3_DMP::readParams]: Failed to read \"a_orient\" param.");
  if (!nh.getParam("c_orient", c_orient)) throw std::ios_base::failure("[SE3_DMP::readParams]: Failed to read \"c_orient\" param.");
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
    io_::read_mat(q_start, in, true);
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

double SE3_DMP::sigmoid(double a, double c, double x) const
{
  return 1 / ( 1 + std::exp(a*(x-c)) );
}

void SE3_DMP::simulate()
{
  readParams();

  int i_end = Pd_data.n_cols - 1;

  robot->update();
  arma::vec P0 = robot->getTaskPosition();
  arma::vec Q0 = robot->getTaskOrientation();

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
  arma::vec Z = arma::vec().zeros(3);
  arma::vec ddP = arma::vec().zeros(3);
  arma::vec Q = Q0;
  arma::vec vRot = arma::vec().zeros(3);
  arma::vec phi = arma::vec().zeros(3);
  arma::vec dvRot = arma::vec().zeros(3);

  arma::vec P_robot = P;
  arma::vec dP_robot = dP;
  arma::vec ddP_robot = ddP;
  arma::vec Q_robot = Q;
  arma::vec vRot_robot = vRot;
  arma::vec dvRot_robot = dvRot;

  arma::vec P_f = P_robot - P;
  arma::vec dP_f = dP_robot - dP;
  arma::vec ddP_f = ddP_robot - ddP;
  arma::vec Q_f = quatProd( Q_robot, quatInv(Q) );
  arma::vec vRot_f = vRot_robot - vRot;
  arma::vec dvRot_f = dvRot_robot - dvRot;

  arma::vec Fext = arma::vec().zeros(6);

  double t_end = T;
  double t0 = t_end;
  can_clock_ptr->setTau(t0);

  int iters = 0;
  clearData();

  // simulate
  while (robot->isOk() && gui->getMode()==MainWindow::RUN_CONTROLLER)
  {
    // data logging
    Time = arma::join_horiz(Time, arma::vec({t}));
    P_data = arma::join_horiz(P_data, P);
    // dP_data = arma::join_horiz(dP_data, dP);
    // ddP_data = arma::join_horiz(ddP_data, ddP);
    Q_data = arma::join_horiz(Q_data, Q);
    // vRot_data = arma::join_horiz(vRot_data, vRot);
    // dvRot_data = arma::join_horiz(dvRot_data, dvRot);
    P_robot_data = arma::join_horiz(P_robot_data, P_robot);
    Q_robot_data = arma::join_horiz(Q_robot_data, Q_robot);
    x_data = arma::join_horiz(x_data, arma::vec{x});
    Fext_data = arma::join_horiz(Fext_data, Fext);

    // Calc admittance
    arma::vec Fext_raw = robot->getTaskWrench();
    Fext =  a_fext_filt*Fext + (1-a_fext_filt)*Fext_raw;
    arma::vec sign_Fext = arma::sign(Fext);
    arma::vec Fext2 = Fext - sign_Fext%Fext_dead_zone;
    Fext = 0.5*(arma::sign(Fext2)+sign_Fext)%arma::abs(Fext2);

    ddP_f = ( Fext.subvec(0,2) - Dp%dP_f - Kp%P_f ) / Mp;
    arma::vec eq_f = quatLog(Q_f);
    dvRot_f = ( Fext.subvec(3,5) - Do%vRot_f - Ko%eq_f ) / Mo;

    // P_f = P_f + dP_f*dt;
    // dP_f = dP_f + ddP_f*dt;
    //
    // Q_f = quatProd( quatExp(vRot_f*dt), Q_f);
    // vRot_f = vRot_f + dvRot_f*dt;
    //
    // dP_robot = dP_f;
    // vRot_robot = vRot_f;

    // calc stopping
    double s_f = sigmoid(a_force, c_force, arma::norm(Fext));
    double s_p = sigmoid(a_pos, c_pos, arma::norm(P_f));
    double s_q = sigmoid(a_orient, c_orient, arma::norm(eq_f)*180/arma::datum::pi);
    double s_tau = s_f*s_p*s_q;

    // Update phase variable
    can_clock_ptr->setTau(t0/s_tau);
    dx = can_clock_ptr->getPhaseDot(x);
    double tau = can_clock_ptr->getTau();

    // double ds_f = sigmoid_dot(a_force, c_force, x)*dx;
    // double ds_p = sigmoid_dot(a_pos, c_pos, x)*dx;
    // double ds_q = sigmoid_dot(a_orient, c_orient, x)*dx;
    // double ds_tau = ds_f*s_p*s_q + s_f*ds_p*s_q + s_f*s_p*ds_q;
    // double tau_dot = -ds_tau*tau0/s_tau^2;

    // DMP simulation
    dmp_p->calcStatesDot(x, P, Z, P0, Pg);
    arma::vec dZ = dmp_p->getDz();
    dP = dmp_p->getDy();
    // ddP = dZ / tau;
    // ddP = (dZ - tau_dot * dP) / tau;

    dmp_o->calcStatesDot(x, Q, phi, Q0, Qg);
    arma::vec dphi = dmp_o->getDphi();
    vRot = dmp_o->getOmega();
    // dvRot = dphi / tau;
    // dvRot = (dphi - tau_dot * vRot) / tau;

    // Numerical integration
    iters++;
    t = t + dt;
    x = x + dx*dt;
    P = P + dP*dt;
    Z = Z + dZ*dt;
    // dP = dP + ddP*dt;
    Q = quatProd( quatExp(vRot*dt), Q);
    phi = phi + dphi*dt;
    // vRot = vRot + dvRot*dt;

    dP_robot = dP + dP_f;
    P_robot = P_robot + dP_robot*dt;
    // ddP_robot = ddP_robot + ddP_f;
    vRot_robot = vRot + vRot_f;
    Q_robot = quatProd( quatExp(vRot_robot*dt), Q_robot);
    // dvRot_robot = dvRot + dvRot_f;

    P_f = P_robot - P;
    dP_f = dP_f + ddP_f*dt;
    Q_f = quatProd( Q_robot, quatInv(Q) );
    vRot_f = vRot_f + dvRot_f*dt;

    arma::vec P_real = robot->getTaskPosition();
    arma::vec Q_real = robot->getTaskOrientation();
    arma::vec V_cmd = arma::join_vert(dP_robot + 0*(P_robot-P_real), vRot_robot + 0.0*quatLog(quatProd(Q_robot,quatInv(Q_real))) );
    robot->setTaskVelocity(V_cmd);
    robot->update();

    // Stopping criteria
    double e_p = arma::norm(P_real - Pg);
    double e_o = arma::norm( quatLog( quatProd(Q_real, quatInv(Qg)) ) )*180/arma::datum::pi;
    if (x>=1.0 && e_p<0.01 && e_o<2) break;
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
    io_::write_mat(x_data, out, true);
    io_::write_mat(P_data, out, true);
    // io_::write_mat(dP_data, out, true);
    // io_::write_mat(ddP_data, out, true);
    io_::write_mat(Q_data, out, true);
    // io_::write_mat(vRot_data, out, true);
    // io_::write_mat(dvRot_data, out, true);
    io_::write_mat(P_robot_data, out, true);
    io_::write_mat(Q_robot_data, out, true);
    io_::write_mat(Fext_data, out, true);
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
  // dP_data.clear();
  // ddP_data.clear();
  Q_data.clear();
  // vRot_data.clear();
  // dvRot_data.clear();
  P_robot_data.clear();
  Q_robot_data.clear();
  x_data.clear();
  Fext_data.clear();

}

arma::vec SE3_DMP::quatProd(const arma::vec &quat1, const arma::vec &quat2) const
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

arma::vec SE3_DMP::quatExp(const arma::vec &v_rot, double zero_tol) const
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

arma::vec SE3_DMP::quatLog(const arma::vec &quat, double zero_tol) const
{
  arma::vec e = quat.subvec(1,3);
  double n = quat(0);

  if (n > 1) n = 1;
  if (n < -1) n = -1;

  arma::vec omega(3);
  double e_norm = arma::norm(e);

  if (e_norm > zero_tol) omega = 2*std::atan2(e_norm,n)*e/e_norm;
  else omega = arma::vec().zeros(3);

  return omega;
}

arma::vec SE3_DMP::quatInv(const arma::vec &quat) const
{
  arma::vec quatI(4);

  quatI(0) = quat(0);
  quatI.subvec(1,3) = - quat.subvec(1,3);

  return quatI;
}

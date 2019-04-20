#include <se3_dmp/robot/lwr4p_robot.h>

#include <ros/package.h>

arma::vec quatProd(const arma::vec &quat1, const arma::vec &quat2)
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

arma::vec quatExp(const arma::vec &v_rot, double zero_tol=1e-16)
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

arma::vec quatLog(const arma::vec &quat, double zero_tol=1e-16)
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

arma::vec quatInv(const arma::vec &quat)
{
  arma::vec quatI(4);

  quatI(0) = quat(0);
  quatI.subvec(1,3) = - quat.subvec(1,3);

  return quatI;
}

LWR4p_Robot::LWR4p_Robot()
{
  N_JOINTS = 7;

  jpos_low_lim = -arma::vec({170, 120, 170, 120, 170, 120, 170});
  jpos_upper_lim = arma::vec({170, 120, 170, 120, 170, 120, 170});

  jnames.resize(N_JOINTS);
  jnames[0] = "lwr_joint_1";
  for (int i=1;i<N_JOINTS;i++)
  {
    jnames[i] = jnames[i-1];
    jnames[i].back()++;
  }

  // Initialize generic robot with the kuka-lwr model
  // std::cerr << "=======> Creating robot...\n";
  robot.reset(new lwr4p::Robot());
  // std::cerr << "=======> Robot created successfully!\n";


  // =========================================================================
  // =========================================================================
  // =========================================================================
  // =========================================================================
  // =========================================================================
  // =========================================================================
  // =========================================================================
  // =========================================================================
  // =========================================================================


  // arma::vec Kp({30, 30, 30});
  // arma::vec Dp({10, 10, 10});
  // arma::vec Ko({0.8, 0.8, 0.8});
  // arma::vec Do({0.3, 0.3, 0.3});
  //
  // robot->setMode(lwr4p::Mode::TORQUE_CONTROL);
  // robot->waitNextCycle();
  //
  // arma::vec q_robot = robot->getJointPosition();
  // arma::vec q_robot_prev = q_robot;
  // arma::vec P0 = this->getTaskPosition();
  // arma::vec Q0 = this->getTaskOrientation();
  // arma::vec Q_robot_prev = Q0;
  // double dt = robot->getControlCycle();
  //
  // std::string path = ros::package::getPath("se3_dmp") + "/out.txt";
  // std::ofstream out(path, std::ios::out);
  //
  // std::cerr << "===>> Main Ctrl loop...\n";
  //
  // int iters = 0;
  //
  // // simulate
  // while (isOk())
  // {
  //   q_robot_prev = q_robot;
  //   q_robot = robot->getJointPosition();
  //   arma::vec dq_robot = (q_robot - q_robot_prev) / dt;
  //   arma::mat Jrobot = robot->getRobotJacobian();
  //   arma::vec Vrobot = Jrobot*dq_robot;
  //
  //   arma::vec dP_robot = Vrobot.subvec(0,2);
  //   arma::vec P_robot = robot->getTaskPosition();
  //   arma::vec vRot_robot = Vrobot.subvec(3,5);
  //   arma::vec Q_robot = getTaskOrientation();
  //   if (arma::dot(Q_robot_prev, Q_robot)<0) Q_robot=-Q_robot;
  //
  //   arma::vec P = P0;
  //   arma::vec dP = arma::vec().zeros(3);
  //   arma::vec Q = Q0;
  //   arma::vec vRot = arma::vec().zeros(3);
  //
  //   arma::vec eo = quatLog( quatProd( Q_robot, quatInv(Q) ) );
  //   arma::vec u = -Jrobot.submat(0, 0, 2, 6).t() * ( Kp%(P_robot-P) + Dp%(dP_robot-dP)  )
  // 		           -Jrobot.submat(3, 0, 5, 6).t() * (Ko%eo + Do%(vRot_robot-vRot) );
  //
  //   if (iters % 100 == 0)
  //   {
  //     out << "=========================================\n";
  //     out << "e_p " << arma::norm(P_robot-P) << "\n"
  //         << "de_p = " << arma::norm(dP_robot-dP) << "\n"
  //         << "e_o = " << arma::norm(eo) << "\n"
  //         << "de_o = " << arma::norm(vRot_robot-vRot) << "\n"
  //         << "u = " << u.t() << "\n";
  //   }
  //
  //   robot->setJointTorque(u);
  //   robot->waitNextCycle();
  // }
  //
  // out.close();
  //
  // exit(-1);


  // =========================================================================
  // =========================================================================
  // =========================================================================
  // =========================================================================
  // =========================================================================
  // =========================================================================
  // =========================================================================
  // =========================================================================
  // =========================================================================



  std::string ft_sensor_ip = "192.168.2.1";
  // std::cerr << "Initializing F/T sensor at ip: " << ft_sensor_ip << "\n";
  ftsensor.init(ft_sensor_ip.c_str());
  ftsensor.setTimeout(1.0);
  ftsensor.setBias();

  ext_stop = false;
  mode.set(Robot::STOPPED);
  cmd_mode.set(mode.get());
  jpos_cmd.set(robot->getJointPosition());

  std::thread(&LWR4p_Robot::commandThread,this).detach();
}

LWR4p_Robot::~LWR4p_Robot()
{

}

void LWR4p_Robot::setMode(const Robot::Mode &mode)
{
  if (mode == cmd_mode.get()) return;

  cmd_mode.set(mode);
  mode_change.wait(); // wait to get notification from commandThread
}

void LWR4p_Robot::commandThread()
{
  arma::mat J;
  arma::vec dq;

  while (isOk())
  {
    Mode new_mode = cmd_mode.get();
    // check if we have to switch mode
    if (new_mode != mode.get())
    {
      switch (new_mode)
      {
        case JOINT_TORQUE_CONTROL:
          robot->setMode(lwr4p::Mode::TORQUE_CONTROL);
          jtorque_cmd.set(arma::vec().zeros(N_JOINTS));
          break;
        case FREEDRIVE:
          robot->setMode(lwr4p::Mode::TORQUE_CONTROL);
          jtorque_cmd.set(arma::vec().zeros(N_JOINTS));
          break;
        case CART_VEL_CTRL:
          robot->setMode(lwr4p::Mode::VELOCITY_CONTROL);
          cart_vel_cmd.set(arma::vec().zeros(6));
          break;
        case IDLE:
          robot->setMode(lwr4p::Mode::POSITION_CONTROL);
          jpos_cmd.set(robot->getJointPosition());
          break;
        case STOPPED:
          robot->setMode(lwr4p::Mode::POSITION_CONTROL);
          robot->setMode(lwr4p::Mode::STOPPED);
          // robot->setExternalStop(true);
          mode.set(new_mode);
          mode_change.notify(); // unblock in case wait was called from another thread
          KRC_tick.notify(); // unblock in case wait was called from another thread
          return;
      }
      mode.set(new_mode);
      mode_change.notify();
    }

    // send command according to current mode
    switch (mode.get())
    {
      case JOINT_TORQUE_CONTROL:
        robot->setJointTorque(jtorque_cmd.get());
        // std::cerr << "jtorque_cmd.get() = " << jtorque_cmd.get().t() << "\n";
        break;
      case CART_VEL_CTRL:
        J = robot->getRobotJacobian();
        dq = arma::pinv(J)*cart_vel_cmd.get();
        // std::cerr << "dq = " << dq.t() << "\n";
        robot->setJointVelocity(dq);
        break;
      case Robot::Mode::FREEDRIVE:
        robot->setJointTorque(jtorque_cmd.get());
        break;
      case Robot::Mode::IDLE:
        robot->setJointPosition(jpos_cmd.get());
        break;
    }

    // sync with KRC
    robot->waitNextCycle();
    KRC_tick.notify();
  }

  mode_change.notify(); // unblock in case wait was called from another thread
  KRC_tick.notify(); // unblock in case wait was called from another thread
}

arma::mat get5thOrder(double t, arma::vec p0, arma::vec pT, double totalTime)
{
  arma::mat retTemp = arma::zeros<arma::mat>(p0.n_rows, 3);

  if (t < 0)
  {
    // before start
    retTemp.col(0) = p0;
  }
  else if (t > totalTime)
  {
    // after the end
    retTemp.col(0) = pT;
  }
  else
  {
    // somewhere betweeen ...
    // position
    retTemp.col(0) = p0 +
                     (pT - p0) * (10 * pow(t / totalTime, 3) -
                     15 * pow(t / totalTime, 4) +
                     6 * pow(t / totalTime, 5));
    // vecolity
    retTemp.col(1) = (pT - p0) * (30 * pow(t, 2) / pow(totalTime, 3) -
                     60 * pow(t, 3) / pow(totalTime, 4) +
                     30 * pow(t, 4) / pow(totalTime, 5));
    // acceleration
    retTemp.col(2) = (pT - p0) * (60 * t / pow(totalTime, 3) -
                     180 * pow(t, 2) / pow(totalTime, 4) +
                     120 * pow(t, 3) / pow(totalTime, 5));
  }

  // return vector
  return retTemp;
}

bool LWR4p_Robot::setJointsTrajectory(const arma::vec &qT, double duration)
{
  // keep last known robot mode
  Robot::Mode prev_mode = this->getMode();
  // start controller
  this->setMode(Robot::IDLE);
  // std::cerr << "[LWR4p_Robot::setJointsTrajectory]: Mode changed to \"IDLE\"!\n";

  // waits for the next tick
  update();

  arma::vec q0 = robot->getJointPosition();
  arma::vec qref = q0;
  // std::cerr << "q0 = " << q0.t()*180/3.14159 << "\n";
  // std::cerr << "duration = " << duration << " sec\n";

  // robot->setMode(lwr4p::Mode::POSITION_CONTROL);
  // initalize time
  double t = 0.0;
  // the main while
  while (t < duration)
  {
    if (!isOk())
    {
      err_msg = "An error occured on the robot!";
      return false;
    }

    // compute time now
    t += getCtrlCycle();
    // update trajectory
    qref = get5thOrder(t, q0, qT, duration).col(0);

    // set joint positions
    jpos_cmd.set(qref);
    //setJointPosition(qref);

    // waits for the next tick
    update();
  }
  // reset last known robot mode
  this->setMode(prev_mode);

  // std::cerr << "[LWR4p_Robot::setJointsTrajectory]: Mode restored to previous mode!\n";

  return true;
}

void LWR4p_Robot::stop()
{
  // setMode(Robot::STOPPED);
  robot->stop();
}

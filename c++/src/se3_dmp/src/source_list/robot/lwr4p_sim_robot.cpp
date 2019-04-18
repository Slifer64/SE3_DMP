#include <se3_dmp/robot/lwr4p_sim_robot.h>

LWR4p_Sim_Robot::LWR4p_Sim_Robot()
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

  task_pose = arma::mat().eye(4,4);
  task_pos = arma::vec().zeros(3);
  task_orient = arma::vec().zeros(4);
  jpos = arma::vec().zeros(N_JOINTS);
  task_wrench = arma::vec().zeros(6);
  Jacob = arma::mat().eye(6, N_JOINTS);

  ext_stop = false;
  is_ok = true;
  ctr_cycle = 0.002;
  mode.set(Robot::STOPPED);
  cmd_mode.set(mode.get());
  jpos_cmd.set(jpos);

  std::thread(&LWR4p_Sim_Robot::commandThread,this).detach();
}

LWR4p_Sim_Robot::~LWR4p_Sim_Robot()
{

}

void LWR4p_Sim_Robot::setMode(const Robot::Mode &mode)
{
  if (mode == cmd_mode.get()) return;

  cmd_mode.set(mode);
  mode_change.wait(); // wait to get notification from commandThread
}

void LWR4p_Sim_Robot::commandThread()
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
          // robot->setMode(lwr4p::Mode::TORQUE_CONTROL);
          jtorque_cmd.set(arma::vec().zeros(N_JOINTS));
          break;
        case FREEDRIVE:
          // robot->setMode(lwr4p::Mode::TORQUE_CONTROL);
          jtorque_cmd.set(arma::vec().zeros(N_JOINTS));
          break;
        case CART_VEL_CTRL:
          // robot->setMode(lwr4p::Mode::TORQUE_CONTROL);
          cart_vel_cmd.set(arma::vec().zeros(6));
          break;
        case IDLE:
          // robot->setMode(lwr4p::Mode::POSITION_CONTROL);
          jpos_cmd.set(jpos);
          break;
        case STOPPED:
          // robot->setMode(lwr4p::Mode::POSITION_CONTROL);
          // robot->setMode(lwr4p::Mode::STOPPED);
          // robot->setExternalStop(true);
          mode.set(new_mode);
          mode_change.notify(); // unblock in case wait was called from another thread
          KRC_tick.notify(); // unblock in case wait was called from another thread
          return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1500)); // simulate delay during Robot's mode change
      mode.set(new_mode);
      mode_change.notify();
    }

    // send command according to current mode
    switch (mode.get())
    {
      case JOINT_TORQUE_CONTROL:
        // robot->setJointTorque(jtorque_cmd.get());
        break;
      case CART_VEL_CTRL:
        J = Jacob;
        dq = arma::pinv(J)*cart_vel_cmd.get();
        jpos = jpos + dq*getCtrlCycle();
        break;
      case Robot::Mode::FREEDRIVE:
        // robot->setJointTorque(jtorque_cmd.get());
        break;
      case Robot::Mode::IDLE:
        jpos = jpos_cmd.get();
        break;
    }

    // sync with KRC
    waitNextCycle();
    KRC_tick.notify();
  }

  mode_change.notify(); // unblock in case wait was called from another thread
  KRC_tick.notify(); // unblock in case wait was called from another thread
}

arma::mat LWR4p_Sim_Robot::get5thOrder(double t, arma::vec p0, arma::vec pT, double totalTime) const
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

bool LWR4p_Sim_Robot::setJointsTrajectory(const arma::vec &qT, double duration)
{
  // keep last known robot mode
  Robot::Mode prev_mode = this->getMode();
  // start controller
  this->setMode(Robot::IDLE);
  // std::cerr << "[LWR4p_Sim_Robot::setJointsTrajectory]: Mode changed to \"IDLE\"!\n";

  // waits for the next tick
  update();

  arma::vec q0 = jpos;
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

  // std::cerr << "[LWR4p_Sim_Robot::setJointsTrajectory]: Mode restored to previous mode!\n";

  return true;
}

void LWR4p_Sim_Robot::stop()
{
  // setMode(Robot::STOPPED);
  // robot->stop();
}

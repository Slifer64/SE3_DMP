#ifndef SE3_DMP_LWR4P_SIM_ROBOT_H
#define SE3_DMP_LWR4P_SIM_ROBOT_H

#include <se3_dmp/robot/robot.h>
#include <lwr4p/Robot.h>

class LWR4p_Sim_Robot: public Robot
{
public:
  LWR4p_Sim_Robot();
  ~LWR4p_Sim_Robot();

  void commandThread();

  int getNumOfJoints() const
  { return N_JOINTS; }

  std::string getErrMsg() const
  { return err_msg; }

  arma::vec getTaskPosition() const
  { return task_pos; }

  arma::vec getTaskOrientation() const
  { return task_orient; }

  arma::vec getTaskForce() const
  { return task_wrench.subvec(0,2); }

  arma::vec getTaskTorque() const
  { return task_wrench.subvec(3,5); }

  arma::vec getTaskWrench() const
  { return task_wrench; }

  arma::vec getJointsPosition() const
  { return jpos; }

  arma::mat getJacobian() const
  { return Jacob; }

  void update()
  { if (isOk()) KRC_tick.wait(); }

  arma::vec getJointsLowerLimits() const
  { return jpos_low_lim; }

  arma::vec getJointsUpperLimits() const
  { return jpos_upper_lim; }

  void stop();

  void setMode(const Robot::Mode &mode);

  double getCtrlCycle() const
  { return ctr_cycle; }

  bool isOk() const
  { return (is_ok && !ext_stop()); }

  void setTaskVelocity(const arma::vec &vel) { cart_vel_cmd.set(vel); }
  bool setJointsTrajectory(const arma::vec &qT, double duration);

  void setExternalStop(bool set) { ext_stop=set; }

  std::vector<std::string> getJointNames() const
  { return jnames; }

private:

  void waitNextCycle() const
  { std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(getCtrlCycle()*1e3))); }

  arma::mat get5thOrder(double t, arma::vec p0, arma::vec pT, double totalTime) const;

  arma::mat task_pose;
  arma::vec task_pos;
  arma::vec task_orient;
  arma::vec jpos;
  arma::vec task_wrench;
  arma::mat Jacob;

  double ctr_cycle;
  bool is_ok;

  MtxVar<bool> ext_stop;

  std::string err_msg;

  int N_JOINTS;

  MtxVar<Mode> cmd_mode;

  Semaphore mode_change;

  arma::vec jpos_low_lim;
  arma::vec jpos_upper_lim;
  std::vector<std::string> jnames;
};

#endif // SE3_DMP_LWR4p_Sim_Robot_H

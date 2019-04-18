#ifndef SE3_DMP_LWR4P_ROBOT_H
#define SE3_DMP_LWR4P_ROBOT_H

#include <se3_dmp/robot/robot.h>
#include <lwr4p/Robot.h>
#include <ati_sensor/ft_sensor.h>

class LWR4p_Robot: public Robot
{
public:
  LWR4p_Robot();
  ~LWR4p_Robot();

  void commandThread();

  int getNumOfJoints() const
  { return N_JOINTS; }

  std::string getErrMsg() const
  {
    return err_msg;
  }

  arma::vec getTaskPosition() const
  { return robot->getTaskPosition(); }

  arma::vec getTaskOrientation() const
  {
    arma::vec task_orient(4);
    arma::mat R = robot->getTaskOrientation();
    task_orient = rotm2quat(R);
    return task_orient;
  }

  arma::vec getTaskForce() const
  { return getTaskWrench().subvec(0,2); }

  arma::vec getTaskTorque() const
  { return getTaskWrench().subvec(3,5); }

  arma::vec getTaskWrench() const
  {
    // static double measurements[6];
    // uint32_t rdt(0),ft(0);
    // (const_cast<ati::FTSensor *>(&ftsensor))->getMeasurements(measurements,rdt,ft);
    // //ftsensor.getMeasurements(measurements,rdt,ft);
    //
    // arma::vec Fext(6);
    // Fext(0) = measurements[0];
    // Fext(1) = measurements[1];
    // Fext(2) = measurements[2];
    // Fext(3) = measurements[3];
    // Fext(4) = measurements[4];
    // Fext(5) = measurements[5];
    //
    // arma::mat R = robot->getTaskOrientation();
    // Fext.subvec(0,2) = R*Fext.subvec(0,2);
    // Fext.subvec(3,5) = R*Fext.subvec(3,5);

    arma::vec Fext(6);
    Fext = robot->getExternalWrench();
    Fext = -Fext;

    return Fext;
  }

  arma::vec getJointsPosition() const
  { return robot->getJointPosition(); }

  arma::mat getJacobian() const
  { return robot->getRobotJacobian(); }

  void update()
  { if (isOk()) KRC_tick.wait(); }

  arma::vec getJointsLowerLimits() const
  { return jpos_low_lim; }

  arma::vec getJointsUpperLimits() const
  { return jpos_upper_lim; }

  void stop();

  void setMode(const Robot::Mode &mode);

  double getCtrlCycle() const
  { return robot->getControlCycle(); }

  bool isOk() const
  { return (robot->isOk() && !ext_stop()); }

  void setJointsTorque(const arma::vec &jtorq) { jtorque_cmd.set(jtorq); }
  void setTaskVelocity(const arma::vec &vel) { cart_vel_cmd.set(vel); }
  bool setJointsTrajectory(const arma::vec &qT, double duration);

  void setExternalStop(bool set) { ext_stop=set; robot->stop(); }

  std::vector<std::string> getJointNames() const
  { return jnames; }

private:
  std::shared_ptr<lwr4p::Robot> robot;
  ati::FTSensor ftsensor;

  MtxVar<bool> ext_stop;

  std::string err_msg;

  int N_JOINTS;

  MtxVar<Mode> cmd_mode;

  Semaphore mode_change;

  arma::vec jpos_low_lim;
  arma::vec jpos_upper_lim;
  std::vector<std::string> jnames;
};

#endif // SE3_DMP_LWR4P_ROBOT_H

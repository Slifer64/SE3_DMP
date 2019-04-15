#ifndef SE3_DMP_ROBOT_H
#define SE3_DMP_ROBOT_H

#include <cstdlib>
#include <exception>
#include <vector>
#include <cstring>
#include <thread>
#include <armadillo>
#include <Eigen/Dense>

#include <se3_dmp/utils.h>

class Robot
{
public:
  /** The control modes that can be applied to the robot. */
  enum Mode
  {
    FREEDRIVE, // freedrive mode (or gravity compensation)
    IDLE, // robot is idle and doesn't move
    STOPPED, // the robot stops
  };

  Robot();
  ~Robot();

  Robot::Mode getMode() const;
  std::string getModeName() const;

  virtual std::string getErrMsg() const = 0;

  virtual int getNumOfJoints() const = 0;

  virtual arma::vec getTaskPosition() const = 0;
  virtual arma::vec getTaskOrientation() const = 0;
  virtual arma::vec getTaskForce() const = 0;
  virtual arma::vec getTaskTorque() const = 0;
  virtual arma::vec getTaskWrench() const = 0;

  virtual arma::vec getJointsPosition() const = 0;
  virtual arma::vec getJointsTorque() const = 0;
  virtual arma::mat getJacobian() const = 0;

  /** Updates the robot state (position, forces, velocities etc.) by reading them
   *  from the actual hardware. Must be called once in each control cycle.
   */
  virtual void update() = 0;

  virtual void stop() = 0;

  virtual void setMode(const Robot::Mode &mode) = 0;

  virtual double getCtrlCycle() const = 0;
  virtual bool isOk() const = 0;

  virtual void commandThread() = 0;

  virtual bool setJointsTrajectory(const arma::vec &qT, double duration) = 0;

  virtual arma::vec getJointsLowerLimits() const = 0;
  virtual arma::vec getJointsUpperLimits() const = 0;

  virtual std::vector<std::string> getJointNames() const = 0;

  virtual void setExternalStop(bool set) = 0;

protected:

  Eigen::Vector4d rotm2quat(Eigen::Matrix3d rotm) const;
  arma::vec rotm2quat(const arma::mat &rotm) const ;

  MtxVar<Mode> mode; // current mode
  std::vector<std::string> mode_name; ///< robot's control mode name

  MtxVar<arma::vec> jpos_cmd;
  MtxVar<arma::vec> jtorque_cmd;

  Semaphore KRC_tick;
};

#endif // SE3_DMP_ROBOT_H

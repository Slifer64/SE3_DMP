#ifndef SE3_DMP_ROBOT_H
#define SE3_DMP_ROBOT_H

#include <cstdlib>
#include <exception>
#include <vector>
#include <cstring>
#include <armadillo>
#include <Eigen/Dense>

#include <se3_dmp/utils.h>

class Robot
{
public:
  /** The control modes that can be applied to the robot. */
  enum Mode
  {
    CART_VEL_CTRL, // Cartesian velocity control
    FREEDRIVE, // freedrive mode
    IDLE, // robot is idle and doesn't move
  };

  Robot();
  ~Robot();

  Robot::Mode getMode() const;
  std::string getModeName() const;

  // ===========================================
  // ========   Virtual functions  =============
  // ===========================================


  virtual void init() = 0;

  virtual double getControlCycle() const = 0;

  virtual bool isOk() const = 0;

  virtual std::string getErrMsg() const;

  virtual void update() = 0;

  virtual void command() = 0;

  virtual void setMode(const Robot::Mode &mode) = 0;

  virtual void setTaskVelocity(const arma::vec &vel) = 0;

  virtual void setJointTrajectory(const arma::vec &qT, double duration) = 0;

  virtual arma::vec getJointPosition() const = 0;

  virtual arma::mat getTaskPose() const = 0;

  virtual arma::vec getTaskPosition() const = 0;

  virtual arma::vec getTaskOrientation() const = 0;

  virtual arma::vec getTaskWrench() = 0;

protected:

  Eigen::Vector4d rotm2quat(Eigen::Matrix3d rotm) const;
  arma::vec rotm2quat(const arma::mat &rotm) const ;

  Mode mode; ///< robot's control mode
  std::vector<std::string> modeName; ///< robot's control mode name
  arma::vec vel_cmd; ///< commanded task velocity expressed in base frame.

  arma::vec Fext_dead_zone; ///< dead zone applied to external force measurement

  std::string err_msg; ///< contains the description of an occured error.

  void readParams(const char *params_file = NULL);

};

#endif // SE3_DMP_ROBOT_H

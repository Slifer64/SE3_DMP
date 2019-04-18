#include <se3_dmp/robot/robot.h>
#include <ros/package.h>
#include <io_lib/parser.h>

Robot::Robot()
{
  mode_name.resize(5);
  mode_name[0] = "JOINT_TORQUE_CONTROL";
  mode_name[1] = "CART_VEL_CTRL";
  mode_name[2] = "FREEDRIVE";
  mode_name[3] = "IDLE";
  mode_name[4] = "STOPPED";
}

Robot::~Robot()
{

}

Robot::Mode Robot::getMode() const
{
  return mode.get();
}

std::string Robot::getModeName() const
{
  return mode_name[getMode()];
}

Eigen::Vector4d Robot::rotm2quat(Eigen::Matrix3d rotm) const
{
    Eigen::Quaternion<double> temp_quat(rotm);
    Eigen::Vector4d quat;
    quat << temp_quat.w(), temp_quat.x(), temp_quat.y(), temp_quat.z();

    quat = quat * (2*(quat(0)>=0)-1); // to avoid discontinuities

    return quat;
}

arma::vec Robot::rotm2quat(const arma::mat &rotm) const
{
  arma::vec quat(4);

  Eigen::Map<const Eigen::Matrix3d> rotm_wrapper(rotm.memptr());
  Eigen::Map<Eigen::Vector4d> quat_wrapper(quat.memptr());
  quat_wrapper = rotm2quat(rotm_wrapper);

  return quat;
}

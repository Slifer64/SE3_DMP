#include <se3_dmp/robot/robot.h>
#include <ros/package.h>
#include <io_lib/parser.h>

Robot::Robot()
{
  modeName.resize(3);
  modeName[0] = "CART_VEL_CTRL";
  modeName[1] = "FREEDRIVE";
  modeName[2] = "IDLE";

  vel_cmd = arma::vec().zeros(6);

  readParams();
}

Robot::~Robot()
{

}

void Robot::readParams(const char *params_file)
{
  std::string path_to_config_file;
  if (params_file != NULL) path_to_config_file = *params_file;
  else path_to_config_file = ros::package::getPath(PACKAGE_NAME)+ "/config/Robot_config.yml";
  as64_::io_::Parser parser(path_to_config_file);

  if (!parser.getParam("Fext_dead_zone", Fext_dead_zone)) Fext_dead_zone = arma::vec().zeros(6);
}

Robot::Mode Robot::getMode() const
{
  return this->mode;
}

std::string Robot::getModeName() const
{
  return modeName[getMode()];
}

std::string Robot::getErrMsg() const
{
  return err_msg;
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

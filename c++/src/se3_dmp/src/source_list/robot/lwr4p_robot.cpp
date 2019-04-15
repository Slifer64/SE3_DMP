#include <se3_dmp/robot/lwr4p_robot.h>

LWR4p_Robot::LWR4p_Robot():N_JOINTS(7)
{
  // Initialize generic robot with the kuka-lwr model
  robot.reset(new lwr4p::Robot());
  std::cout << "Robot created successfully!\n";

  std::string ft_sensor_ip = "192.168.2.1";
  std::cout << "Initializing F/T sensor at ip: " << ft_sensor_ip << "\n";
  ftsensor.init(ft_sensor_ip.c_str());
  ftsensor.setTimeout(1.0);
  ftsensor.setBias();
}

LWR4p_Robot::~LWR4p_Robot()
{

}

bool LWR4p_Robot::isOk() const
{
  return (robot->isOk() || robot->getMode()==lwr4p::Mode::STOPPED);
}

void LWR4p_Robot::init()
{
  vel_cmd = arma::vec().zeros(6);

  readParams();

  ftsensor.setBias();

  this->update();
}

double LWR4p_Robot::getControlCycle() const
{
  return robot->getControlCycle();
}

void LWR4p_Robot::update()
{
  robot->waitNextCycle();
}

void LWR4p_Robot::command()
{
  arma::mat J;
  arma::vec dq;

  switch (this->getMode())
  {
    case Robot::Mode::CART_VEL_CTRL:
      J = robot->getRobotJacobian();
      dq = arma::pinv(J)*vel_cmd;
      robot->setJointVelocity(dq);
      break;
    case Robot::Mode::FREEDRIVE:
      robot->setJointTorque(arma::vec().zeros(N_JOINTS));
      break;
    case Robot::Mode::IDLE:
      // do nothing for kuka
      break;
  }
}

void LWR4p_Robot::setMode(const Robot::Mode &mode)
{
  if (mode == this->getMode()) return;

  switch (mode){
    case CART_VEL_CTRL:
      robot->setMode(lwr4p::Mode::VELOCITY_CONTROL);
      PRINT_INFO_MSG("Robot set in CART_VEL_CTRL MODE.\n");
      break;
    case FREEDRIVE:
      robot->setMode(lwr4p::Mode::TORQUE_CONTROL);
      PRINT_INFO_MSG("Robot set in FREEDRIVE MODE.\n");
      break;
    case IDLE:
      // for (int i=0;i<10;i++)
      // {
      //   robot->setJointVelocity(arma::vec().zeros(7));
      //   update();
      // }
      // robot->setMode(lwr4p::Mode::POSITION_CONTROL);
      robot->setMode(lwr4p::Mode::STOPPED);
      PRINT_INFO_MSG("Robot set in IDLE MODE.\n");
      break;
  }
  this->mode = mode;
}

void LWR4p_Robot::stop()
{
  robot->setMode(lwr4p::Mode::STOPPED);
}

void LWR4p_Robot::setTaskVelocity(const arma::vec &vel)
{
  if (this->getMode() != Robot::Mode::CART_VEL_CTRL)
  {
    throw std::runtime_error("[ERROR] LWR4p_Robot::setTaskVelocity: Current mode is \"" + getModeName() + "\".\n");
  }

  vel_cmd = vel;
}

void LWR4p_Robot::setJointTrajectory(const arma::vec &qT, double duration)
{
  robot->setJointTrajectory(qT, duration);
}

arma::vec LWR4p_Robot::getJointPosition() const
{
  return robot->getJointPosition();
}

arma::mat LWR4p_Robot::getTaskPose() const
{
  return robot->getTaskPose();
}

arma::vec LWR4p_Robot::getTaskPosition() const
{
  arma::vec task_pos = robot->getTaskPosition();
  return task_pos;
}

arma::vec LWR4p_Robot::getTaskOrientation() const
{
  arma::vec task_orient(4);
  arma::mat R = robot->getTaskOrientation();
  task_orient = rotm2quat(R);
  return task_orient;
}

arma::vec LWR4p_Robot::getTaskWrench()
{
  static double measurements[6];
  uint32_t rdt(0),ft(0);
  ftsensor.getMeasurements(measurements,rdt,ft);

  arma::vec Fext(6);
  Fext(0) = measurements[0];
  Fext(1) = measurements[1];
  Fext(2) = measurements[2];
  Fext(3) = measurements[3];
  Fext(4) = measurements[4];
  Fext(5) = measurements[5];

  arma::mat R = robot->getTaskOrientation();
  Fext.subvec(0,2) = R*Fext.subvec(0,2);
  Fext.subvec(3,5) = R*Fext.subvec(3,5);

  // Fext = robot->getExternalWrench();
  // Fext = -Fext;

  arma::vec sign_Fext = arma::sign(Fext);
  arma::vec Fext2 = Fext - sign_Fext%Fext_dead_zone;
  Fext2 = 0.5*(arma::sign(Fext2)+sign_Fext)%arma::abs(Fext2);

  return Fext2;
}
